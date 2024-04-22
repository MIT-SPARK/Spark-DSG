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
#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "spark_dsg/dynamic_scene_graph.h"

namespace spark_dsg::io {

// Define the current project and version when serializing data with this
// implementation.
inline const std::string CURRENT_PROJECT_NAME = "main";

// Which project names are compatible to load given the current project. <current,
// {compatible, ...}>. Projects are always assumed to be compatible with themselves.
inline const std::unordered_map<std::string, std::unordered_set<std::string>>
    PROJECT_COMPATIBILITY = {};

// Version information for compatibility (and potential backwards compatibility)
// within a project.
struct Version {
  uint8_t major = 0u;
  uint8_t minor = 0u;
  uint8_t patch = 0u;

  // Constructors.
  Version() = default;
  Version(uint8_t _major, uint8_t _minor, uint8_t _patch)
      : major(_major), minor(_minor), patch(_patch) {}

  // Comparison operators.
  bool operator==(const Version& other) const;
  bool operator!=(const Version& other) const { return !(*this == other); }
  bool operator<(const Version& other) const;
  bool operator>(const Version& other) const { return other < *this; }
  bool operator<=(const Version& other) const { return !(*this > other); }
  bool operator>=(const Version& other) const { return !(*this < other); }

  // Human readable string representation.
  std::string toString() const;
};

/**
 * @brief Meta information to be saved at the beginning of a binary file to make sure
 * we're reading meaningful data, allow comparing potentially different projects and
 * versions of the serialized data and ensure or warn about version compatibility.
 */
struct FileHeader {
  // The file type identifier to make sure we're reading compatible binary data.
  //           ! DO NOT CHANGE THIS VALUE !
  // (Otherwise all older files will be unreadable.)
  inline static const std::string IDENTIFIER_STRING = "SPARK_DSG";

  // String identifier for different variants of spark-dsg that may not be compatible.
  std::string project_name = "main";

  // Version of the library when saved.
  Version version;

  // Access to specific versions.
  static FileHeader current();
  static FileHeader legacy();

  // Serilization.
  std::vector<uint8_t> serializeToBinary() const;
  static std::optional<FileHeader> deserializeFromBinary(
      const std::vector<uint8_t>& buffer, size_t* offset = nullptr);

  // Stringify.
  std::string toString() const;
};

/**
 * @brief Warn the user about loading a file with an old header, which may be
 * discontinued in the future.
 * @param header The header of the loaded file.
 */
void warnOutdatedHeader(const FileHeader& header);

/**
 * @brief Check if the loaded file is compatible with the current version of spark-dsg.
 * @param loaded The header of the loaded file.
 * @param current Optional: Specific version to compare against. Defaults to the
 * current.
 */
void checkCompatibility(const FileHeader& loaded,
                        const FileHeader& current = FileHeader::current());

void checkProjectCompatibility(const FileHeader& loaded,
                               const FileHeader& current = FileHeader::current());

/**
 * @brief Global access to the header currently used for de-serialization. This will
 * always be set to the header of a file if a file is loaded. It will be set to the
 * current version if other serialized data is being passed around.
 */
struct GlobalInfo {
 public:
  /**
   * @brief Get the current header used for de-serialization.
   */
  static const FileHeader& loadedHeader() { return loaded_header_; };

  /**
   * @brief Get whether a warning about loading a legacy file has already been issued
   * and set the flag to true.
   */
  static bool warnedLegacy();

  /**
   * @brief Set the current header used for de-serialization.
   */
  struct ScopedInfo {
    explicit ScopedInfo(const FileHeader& header) {
      loaded_header_ = header;
      warned_legacy_ = false;
    }

    ~ScopedInfo() { loaded_header_ = FileHeader::current(); }
  };

 private:
  GlobalInfo() = default;
  thread_local inline static FileHeader loaded_header_ = FileHeader::current();
  thread_local inline static bool warned_legacy_ = false;
};

}  // namespace spark_dsg::io
