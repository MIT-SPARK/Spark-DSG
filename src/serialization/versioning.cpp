/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "spark_dsg/serialization/versioning.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "spark_dsg/logging.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg_version.h"

namespace spark_dsg::io {

void checkCompatibility(const FileHeader& loaded, const FileHeader& current) {
  // Check whether this is a legacy file. We support binary serialziation compatibility
  // from Spark DSG v1.0.2 forward, older versions should be converted to JSON.
  if (loaded.version < Version(1, 0, 2)) {
    throw std::runtime_error(
        "Attempted to load invalid binary file: the loaded file was created with an "
        "unsupported "
        "legacy version of Spark DSG (" +
        loaded.version.toString() +
        "). Please convert the file to JSON and save it again to update to the "
        "current encoding (" +
        current.version.toString() + ").");
  }

  // Check the project name.
  checkProjectCompatibility(loaded, current);

  // Add version compatibility checks if needed. Currently all versons are implemented
  // to be backwards compatible.
}

void checkProjectCompatibility(const FileHeader& loaded, const FileHeader& current) {
  // Check for identical projects.
  if (loaded.project_name == current.project_name) {
    return;
  }

  // Check for known projects.
  // NOTE(lschmid): Modifications for external projects will always be breaking
  // changes that are unknown to, so we employ a hard check here. Alternatively, this
  // distinction could also be more fine graind for known projects.
  const auto it = PROJECT_COMPATIBILITY.find(current.project_name);
  if (it == PROJECT_COMPATIBILITY.end()) {
    std::stringstream msg;
    msg << "Attempted to load invalid binary file: the loaded file was created with an "
           "incompatible project ("
        << loaded.project_name << ") to the current project (" << current.project_name
        << ").";
    throw(std::runtime_error(msg.str()));
  }
  const auto& compatible_projects = it->second;

  const auto it2 = compatible_projects.find(loaded.project_name);
  if (it2 == compatible_projects.end()) {
    std::stringstream msg;
    msg << "Attempted to load invalid binary file: the loaded file was created with an "
           "incompatible project ("
        << loaded.project_name << ") to the current project (" << current.project_name
        << ").";
    throw(std::runtime_error(msg.str()));
  }
}

bool Version::operator==(const Version& other) const {
  return major == other.major && minor == other.minor && patch == other.patch;
}

bool Version::operator<(const Version& other) const {
  if (major < other.major) {
    return true;
  } else if (major == other.major) {
    if (minor < other.minor) {
      return true;
    } else if (minor == other.minor) {
      return patch < other.patch;
    }
  }
  return false;
}

std::string Version::toString() const {
  std::stringstream ss;
  ss << static_cast<int>(major) << "." << static_cast<int>(minor) << "."
     << static_cast<int>(patch);
  return ss.str();
}

std::vector<uint8_t> FileHeader::serializeToBinary() const {
  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(IDENTIFIER_STRING);
  serializer.write(*this);
  return buffer;
}

FileHeader FileHeader::current() {
  FileHeader header;
  header.project_name = CURRENT_PROJECT_NAME;
  header.version.major = SPARK_DSG_VERSION_MAJOR;
  header.version.minor = SPARK_DSG_VERSION_MINOR;
  header.version.patch = SPARK_DSG_VERSION_PATCH;
  return header;
}

FileHeader FileHeader::legacy() {
  FileHeader header;
  header.project_name = "main";
  header.version = Version(1, 0, 0);
  return header;
}

std::string FileHeader::toString() const {
  return project_name + " v" + version.toString();
  std::stringstream ss;
}

void warnOutdatedHeader(const FileHeader& header) {
  if (GlobalInfo::warnedLegacy()) {
    return;
  }
  SG_LOG_DEV << "[SPARK-DSG] [WARNING] Loading file with outdated encoding ("
             << header.toString()
             << "). This format may be discontinued in the future. For optimal "
                "preservation and performance load the file "
                "and save it again to update to the current encoding ("
             << FileHeader::current().toString() << ").";
}

// TODO(nathan) this and the header write might belong in file_io instead
std::optional<FileHeader> FileHeader::deserializeFromBinary(
    const std::vector<uint8_t>& buffer, size_t* offset) {
  // Check the buffer is valid.
  serialization::BinaryDeserializer deserializer(buffer);
  if (deserializer.getCurrType() != serialization::PackType::ARR32) {
    return std::nullopt;
  }

  std::string identifier;
  deserializer.read(identifier);
  if (identifier != IDENTIFIER_STRING) {
    return std::nullopt;
  }

  // Deserialize the header.
  FileHeader header;
  deserializer.read(header);

  if (offset) {
    *offset = deserializer.pos();
  }
  return header;
}

bool GlobalInfo::warnedLegacy() {
  const bool already_warned = warned_legacy_;
  warned_legacy_ = true;
  return already_warned;
}

}  // namespace spark_dsg::io
