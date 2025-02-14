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
#include <nlohmann/json.hpp>
#include <optional>

#include "spark_dsg/node_attributes.h"
#include "spark_dsg/spark_dsg_fwd.h"

namespace spark_dsg {

/**
 * @brief Inverse mapping between numerical labels and category names
 *
 * Generally intended to be instantiated and stored in the scene graph metadata (on a
 * per-layer basis)
 */
class Labelspace {
 public:
  Labelspace() = default;

  /**
   * @brief Construct the label space from a mapping between numerical labels and
   * category names
   */
  explicit Labelspace(const std::map<SemanticLabel, std::string>& label_to_names);

  /**
   * @brief Construct the labelspace from names (assumes the numerical labels correspond
   * to vector indices)
   */
  explicit Labelspace(const std::vector<std::string>& names);

  /**
   * @brief Pull the labelspace from scene graph metadata
   */
  static Labelspace fromMetadata(const DynamicSceneGraph& graph,
                                 LayerId layer,
                                 PartitionId partition = 0);

  /**
   * @brief Pull the labelspace from scene graph metadata
   */
  static Labelspace fromMetadata(const DynamicSceneGraph& graph,
                                 const std::string& name);

  /**
   * @brief Get whether or not the label space is populated
   */
  bool empty() const;

  /**
   * @brief Labelspaces are valid if they aren't empty
   */
  inline operator bool() const { return !empty(); }

  /**
   * @brief Get the corresponding category name to the label if it exists
   */
  std::optional<std::string> getCategory(SemanticLabel label) const;

  /**
   * @brief Get the corresponding label to the name if it exists.
   */
  std::optional<SemanticLabel> getLabel(const std::string& category) const;

  /**
   * @brief Utility function to lookup corresponding category name to node
   */
  std::string getCategory(const SemanticNodeAttributes& attrs,
                          const std::string& unknown_name = "UNKNOWN") const;

  /**
   * @brief Save the label space to metadata
   */
  void save(DynamicSceneGraph& graph, LayerId layer, PartitionId partition = 0) const;

  /**
   * @brief Save the label space to metadata
   */
  void save(DynamicSceneGraph& graph, const std::string& name) const;

 private:
  std::map<SemanticLabel, std::string> label_to_name_;
  std::map<std::string, SemanticLabel> name_to_label_;

 public:
  /**
   * @brief Get constant reference to label-name mapping
   */
  const std::map<SemanticLabel, std::string>& labels_to_names() const {
    return label_to_name_;
  }

  /**
   * @brief Get constant reference to label-name mapping
   */
  const std::map<std::string, SemanticLabel>& names_to_labels() const {
    return name_to_label_;
  }
};

}  // namespace spark_dsg
