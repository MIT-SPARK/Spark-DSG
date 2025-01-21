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

#include "spark_dsg/label_space.h"

#include "spark_dsg/dynamic_scene_graph.h"

namespace spark_dsg {

using LabelNameMap = std::map<SemanticLabel, std::string>;
using NameLabelMap = std::map<std::string, SemanticLabel>;

namespace {

inline NameLabelMap invert(const LabelNameMap& original) {
  NameLabelMap inverted;
  std::transform(original.begin(),
                 original.end(),
                 std::inserter(inverted, inverted.end()),
                 [](const auto& label_name_pair) {
                   return std::make_pair(label_name_pair.second, label_name_pair.first);
                 });
  return inverted;
}

inline LabelNameMap index(const std::vector<std::string>& names) {
  SemanticLabel label = 0;
  LabelNameMap result;
  std::transform(names.begin(),
                 names.end(),
                 std::inserter(result, result.end()),
                 [&label](const auto& name) {
                   const auto label_name_pair = std::make_pair(label, name);
                   ++label;
                   return label_name_pair;
                 });
  return result;
}

}  // namespace

LabelSpace::LabelSpace(const std::map<SemanticLabel, std::string>& label_to_names)
    : label_to_name_(label_to_names), name_to_label_(invert(label_to_names)) {}

LabelSpace::LabelSpace(const std::vector<std::string>& names)
    : LabelSpace(index(names)) {}

LabelSpace LabelSpace::fromMetadata(const DynamicSceneGraph& graph,
                                    LayerId layer,
                                    PartitionId partition) {
  const auto& metadata = graph.metadata();
  const auto labelspace_node = metadata.find("labelspaces");
  if (labelspace_node == metadata.end()) {
    return {};
  }

  const auto layer_node = labelspace_node->find(std::to_string(layer));
  if (layer_node == labelspace_node->end()) {
    return {};
  }

  const auto partition_node = layer_node->find(std::to_string(partition));
  if (partition_node == layer_node->end()) {
    return {};
  }

  const auto mapping = partition_node->get<LabelNameMap>();
  return LabelSpace(mapping);
}

bool LabelSpace::empty() const { return label_to_name_.empty(); }

void LabelSpace::save(DynamicSceneGraph& graph,
                      LayerId layer,
                      PartitionId partition) const {
  graph.metadata.add(nlohmann::json{
      "labelspaces",
      {std::to_string(layer), {std::to_string(partition), label_to_name_}},
  });
}

}  // namespace spark_dsg
