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

#include <iostream>
#include <sstream>

#include "spark_dsg/serialization/binary_serialization.h"
#include "spark_dsg_version.h"

namespace spark_dsg::io {

void read_binary(const serialization::BinaryDeserializer& s, Version& version) {
  s.read(version.major);
  s.read(version.minor);
  s.read(version.patch);
}

void write_binary(serialization::BinarySerializer& s, const Version& version) {
  s.write(version.major);
  s.write(version.minor);
  s.write(version.patch);
}

Version::Version() = default;

Version::Version(uint8_t _major, uint8_t _minor, uint8_t _patch) {
  major = _major;
  minor = _minor;
  patch = _patch;
}

std::string Version::toString() const {
  std::stringstream ss;
  ss << static_cast<int>(major) << "." << static_cast<int>(minor) << "."
     << static_cast<int>(patch);
  return ss.str();
}

Version Version::current() {
  return {SPARK_DSG_VERSION_MAJOR, SPARK_DSG_VERSION_MINOR, SPARK_DSG_VERSION_PATCH};
}

Version Version::min_supported() { return {1, 1, 2}; }

FileHeader FileHeader::current() { return {Version::current()}; }

std::string FileHeader::toString() const {
  return std::string(PROJECT_NAME) + " v" + version.toString();
}

std::vector<uint8_t> FileHeader::serialize() const {
  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(std::string(IDENTIFIER_STRING));
  serializer.write(std::string(PROJECT_NAME));
  serializer.write(version);
  return buffer;
}

std::optional<FileHeader> FileHeader::deserialize(const std::vector<uint8_t>& buffer,
                                                  size_t* offset) {
  serialization::BinaryDeserializer deserializer(buffer);
  if (deserializer.getCurrType() != serialization::PackType::ARR32) {
    return std::nullopt;  // currently strings get packed as dynamic arrays
  }

  std::string identifier;
  deserializer.read(identifier);
  if (identifier != IDENTIFIER_STRING) {
    return std::nullopt;
  }

  std::string project_name;
  deserializer.read(project_name);
  if (project_name != PROJECT_NAME) {
    return std::nullopt;
  }

  FileHeader header;
  deserializer.read(header.version);
  if (offset) {
    *offset = deserializer.pos();
  }

  return header;
}

void GlobalInfo::warnOutdated() {
  if (warned_legacy_) {
    return;
  }

  warned_legacy_ = true;
  const auto ver = loaded_version_.toString();
  if (use_short_message) {
    std::cout << "Loading file with encoding " << ver << " (current "
              << Version::current().toString() << ")" << std::endl;
  } else {
    std::cerr << "[SPARK-DSG] [WARNING] Loading file with outdated encoding (" << ver
              << "). This format may be discontinued in the future. For optimal "
                 "preservation and performance load the file "
                 "and save it again to update to the current encoding ("
              << Version::current().toString() << ")." << std::endl;
  }
}

const Version& GlobalInfo::loadedVersion() { return loaded_version_; };

}  // namespace spark_dsg::io
