# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.
#
#
"""The Spark-DSG package."""

import json
import types
import warnings

from spark_dsg._dsg_bindings import *
from spark_dsg._dsg_bindings import (
    BoundingBoxType,
    DsgLayers,
    DynamicSceneGraph,
    EdgeAttributes,
    LayerKey,
    LayerView,
    NodeAttributes,
    SceneGraphLayer,
    compute_ancestor_bounding_box,
)
from spark_dsg.torch_conversion import scene_graph_layer_to_torch, scene_graph_to_torch


def add_bounding_boxes_to_layer(
    graph, layer_id, child_layer=DsgLayers.PLACES, bbox_type=BoundingBoxType.AABB
):
    """
    Add computed bounding boxes to the node attributes in the graph.

    Computes the bounding box from the centroids of the children. The computed
    bounding box is axis-aligned unless it is empty (in which case it is invalid).

    Args:
        graph (spark_dsg._dsg_bindings.DynamicSceneGraph): scene graph
        layer_id (int): layer to add bindings to
    """
    layer = graph.get_layer(layer_id)
    depth = graph.get_layer_key(layer_id).layer - graph.get_layer_key(child_layer).layer
    assert depth > 0
    for node in layer.nodes:
        bbox = compute_ancestor_bounding_box(
            graph, node.id.value, depth=depth, bbox_type=bbox_type
        )
        node.attributes.bounding_box = bbox


def _add_metadata_interface(obj):
    class Metadata:
        def __init__(self, metadata):
            self._metadata = metadata

        def get(self):
            data_str = self._metadata._get()
            metadata = json.loads(data_str)
            metadata = dict() if metadata is None else metadata
            return types.MappingProxyType(metadata)

        def set(self, obj):
            self._metadata._set(json.dumps(obj))

        def add(self, obj):
            self._metadata._add(json.dumps(obj))

    obj.metadata = property(lambda x: Metadata(x._metadata))


def _hash_layerkey(key):
    return hash((key.layer, key.partition))


def _get_layer_id(graph, name):
    warnings.warn(
        "'get_layer_id' is deprecated. Please use 'get_layer_key'",
        DeprecationWarning,
        stacklevel=2,
    )
    return graph.get_layer_key(name)


LayerKey.__hash__ = _hash_layerkey

_add_metadata_interface(DynamicSceneGraph)
_add_metadata_interface(NodeAttributes)
_add_metadata_interface(EdgeAttributes)

DynamicSceneGraph.get_layer_id = _get_layer_id
DynamicSceneGraph.to_torch = scene_graph_to_torch
SceneGraphLayer.to_torch = scene_graph_layer_to_torch
LayerView.to_torch = scene_graph_layer_to_torch
