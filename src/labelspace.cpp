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

#include "spark_dsg/labelspace.h"

#include <iostream>

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

inline std::string layerInfoToKey(LayerId layer, PartitionId partition) {
  return "_l" + std::to_string(layer) + "p" + std::to_string(partition);
}

}  // namespace

Labelspace::Labelspace(const std::map<SemanticLabel, std::string>& label_to_names)
    : label_to_name_(label_to_names), name_to_label_(invert(label_to_names)) {}

Labelspace::Labelspace(const std::vector<std::string>& names)
    : Labelspace(index(names)) {}

Labelspace Labelspace::fromMetadata(const DynamicSceneGraph& graph,
                                    LayerId layer,
                                    PartitionId partition) {
  return fromMetadata(graph, layerInfoToKey(layer, partition));
}

Labelspace Labelspace::fromMetadata(const DynamicSceneGraph& graph,
                                    const std::string& name) {
  const auto& metadata = graph.metadata();
  const auto labelspace_node = metadata.find("labelspaces");
  if (labelspace_node == metadata.end()) {
    return {};
  }

  const auto mapping_node = labelspace_node->find(name);
  if (mapping_node == labelspace_node->end()) {
    return {};
  }

  const auto mapping = mapping_node->get<LabelNameMap>();
  return Labelspace(mapping);
}

bool Labelspace::empty() const { return label_to_name_.empty(); }

std::optional<std::string> Labelspace::getCategory(SemanticLabel label) const {
  const auto iter = label_to_name_.find(label);
  return iter == label_to_name_.end() ? std::nullopt
                                      : std::optional<std::string>(iter->second);
}

std::optional<SemanticLabel> Labelspace::getLabel(const std::string& name) const {
  const auto iter = name_to_label_.find(name);
  return iter == name_to_label_.end() ? std::nullopt
                                      : std::optional<SemanticLabel>(iter->second);
}

std::string Labelspace::getCategory(const SemanticNodeAttributes& attrs,
                                    const std::string& unknown_name) const {
  return getCategory(attrs.semantic_label).value_or(unknown_name);
}

void Labelspace::save(DynamicSceneGraph& graph,
                      LayerId layer,
                      PartitionId partition) const {
  save(graph, layerInfoToKey(layer, partition));
}

void Labelspace::save(DynamicSceneGraph& graph, const std::string& name) const {
  auto entry = nlohmann::json::object();
  entry["labelspaces"] = nlohmann::json::object();
  entry["labelspaces"][name] = label_to_name_;
  graph.metadata.add(entry);
}

}  // namespace spark_dsg
