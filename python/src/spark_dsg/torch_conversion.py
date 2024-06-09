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
"""
Module handling conversion between scene graphs and pytorch geometric.

Includes specific conversion functions for both homogeneous and heterogeneous
graphs. Note that `DynamicSceneGraph.to_torch()` calls into the relevant
homogeneous or heterogeneous conversion function.
"""
from spark_dsg._dsg_bindings import (
    DynamicSceneGraph,
    SceneGraphLayer,
    SceneGraphNode,
    SceneGraphEdge,
    DsgLayers,
    LayerView,
)
from typing import Callable, Optional, Dict, Union
import numpy as np
import importlib


NodeConversionFunc = Callable[[DynamicSceneGraph, SceneGraphNode], np.ndarray]
EdgeConversionFunc = Callable[[DynamicSceneGraph, SceneGraphEdge], np.ndarray]


DEFAULT_LAYER_MAP = {
    DsgLayers.OBJECTS: "objects",
    DsgLayers.PLACES: "places",
    DsgLayers.ROOMS: "rooms",
    DsgLayers.BUILDINGS: "buildings",
}


def _centroid_bbx_embedding(G, x) -> NodeConversionFunc:
    return np.hstack(
        (
            x.attributes.position,
            x.attributes.bounding_box.dimensions,
        )
    )


def _get_edge_name_map(layer_name_map: Dict[int, str], force_hierarchy: bool = True):
    edge_name_map = {}

    def _get_edge_name(l1, l2, n1, n2):
        if force_hierarchy and l2 > l1:
            return n2, f"{n2}_to_{n1}", n1
        else:
            return n1, f"{n1}_to_{n2}", n2

    for l1, n1 in layer_name_map.items():
        edge_name_map[l1] = {
            l2: _get_edge_name(l1, l2, n1, n2) for l2, n2 in layer_name_map.items()
        }

    return edge_name_map


def _get_torch():
    torch = None
    torch_geometric = None
    try:
        torch = importlib.import_module("torch")
    except ImportError:
        raise ValueError("pytorch not found. conversion disabled")

    try:
        torch_geometric = importlib.import_module("torch_geometric")
    except ImportError:
        raise ValueError("pytorch geometric not found. conversion disabled")

    return torch, torch_geometric


def _get_directed_edge(G, edge, id_map):
    if G.get_node(edge.source).layer < G.get_node(edge.target).layer:
        return id_map[edge.target], id_map[edge.source]
    else:
        return id_map[edge.source], id_map[edge.target]


def scene_graph_layer_to_torch(
    G: Union[SceneGraphLayer, LayerView],
    node_converter: NodeConversionFunc,
    edge_converter: Optional[EdgeConversionFunc] = None,
    double_precision: bool = False,
):
    """
    Convert a scene graph layer to a homogeneous pytorch geometric data structure.

    Args:
        G: scene graph layer to convert
        node_converter: function to generate input node features
        edge_converter: optional function to generate input edge features
        double_precision: whether or not output data attributes have double precision.

    Raises:
        ValueError: If pytorch geometric can't be found for the conversion

    Returns:
        pytorch_geometric.Data: homogeneous pytorch_geometric graph representing the
            scene graph layer.
    """
    torch, torch_geometric = _get_torch()

    # output torch tensor data types
    if double_precision:
        dtype_float = torch.float64
    else:
        dtype_float = torch.float32

    N = G.num_nodes()

    node_features = []
    node_positions = torch.zeros((N, 3), dtype=torch.float64)
    id_map = {}

    for node in G.nodes:
        idx = len(node_features)
        node_positions[idx, :] = torch.tensor(
            np.squeeze(node.attributes.position), dtype=dtype_float
        )
        node_features.append(node_converter(G, node))
        id_map[node.id.value] = idx

    node_features = torch.tensor(np.array(node_features), dtype=dtype_float)

    edge_index = torch.zeros((2, G.num_edges()))
    edge_features = []
    for idx, edge in enumerate(G.edges):
        edge_index[:, idx] = torch.tensor(_get_directed_edge(G, edge, id_map))

        if edge_converter is not None:
            edge_features.append(edge_converter(G, edge))

    if edge_converter is not None:
        edge_features = torch.tensor(np.array(edge_features), dtype_float)

    if edge_index.size(dim=1) > 0:
        if edge_converter is None:
            edge_index = torch_geometric.utils.to_undirected(edge_index)
        else:
            edge_index, edge_features = torch_geometric.utils.to_undirected(
                edge_index, edge_features
            )

    return torch_geometric.data.Data(
        x=node_features,
        edge_index=edge_index,
        edge_attr=None if edge_converter is None else edge_features,
        pos=node_positions,
    )


def scene_graph_to_torch_homogeneous(
    G: DynamicSceneGraph,
    node_converter: NodeConversionFunc = _centroid_bbx_embedding,
    edge_converter: Optional[EdgeConversionFunc] = None,
    is_undirected: bool = True,
    double_precision: bool = False,
    **kwargs,
):
    """
    Convert a scene graph to a homogeneous pytorch geometric data structure.

    Args:
        G: scene graph to convert
        node_converter: function to generate input node features
        edge_converter: optional function to generate input edge features
        is_undirected: whether or not the graph should be treated as undirected
        double_precision: whether or not output data attributes have double precision.
        **kargs: absorbing keyword arguments for heterogeneous conversion arguments

    Raises:
        ValueError: If pytorch geometric can't be found for the conversion

    Returns:
        pytorch_geometric.Data: homogeneous pytorch_geometric graph representing the
            scene graph.
    """
    torch, torch_geometric = _get_torch()

    # output torch tensor data types
    if double_precision:
        dtype_int = torch.int64
        dtype_float = torch.float64
    else:
        dtype_int = torch.int32
        dtype_float = torch.float32

    N = G.num_static_nodes()

    node_features = []
    node_positions = torch.zeros((N, 3), dtype=torch.float64)
    node_masks = {x.id: torch.zeros(N, dtype=torch.bool) for x in G.layers}
    node_labels = []
    node_ids = []
    id_map = {}

    for node in G.nodes:
        idx = len(node_features)
        node_masks[node.layer][idx] = True
        node_positions[idx, :] = torch.tensor(
            np.squeeze(node.attributes.position), dtype=dtype_float
        )
        node_features.append(node_converter(G, node))
        id_map[node.id.value] = idx
        node_ids.append(node.id.value)
        node_labels.append(node.attributes.semantic_label)

    node_features = torch.tensor(np.array(node_features), dtype=dtype_float)
    node_labels = torch.tensor(np.array(node_labels), dtype=dtype_int)
    node_ids = torch.tensor(np.array(node_ids), dtype=torch.int64)

    edge_index = torch.zeros((2, G.num_static_edges()), dtype=torch.int64)
    edge_features = []
    for idx, edge in enumerate(G.edges):
        edge_index[:, idx] = torch.tensor(_get_directed_edge(G, edge, id_map))

        if edge_converter is not None:
            edge_features.append(edge_converter(G, edge))

    if edge_converter is not None:
        edge_features = torch.tensor(np.array(edge_features), dtype=dtype_float)

    if edge_index.size(dim=1) > 0 and is_undirected:
        if edge_converter is None:
            edge_index = torch_geometric.utils.to_undirected(edge_index)
        else:
            edge_index, edge_features = torch_geometric.utils.to_undirected(
                edge_index, edge_features
            )

    data = torch_geometric.data.Data(
        x=node_features,
        label=node_labels,
        edge_index=edge_index,
        edge_attr=None if edge_converter is None else edge_features,
        pos=node_positions,
        node_masks=node_masks,
        node_ids=node_ids,
    )

    return data


def scene_graph_to_torch_heterogeneous(
    G: DynamicSceneGraph,
    node_converter: NodeConversionFunc = _centroid_bbx_embedding,
    edge_converter: Optional[EdgeConversionFunc] = None,
    layer_name_map: Optional[Dict[int, str]] = None,
    is_undirected: bool = True,
    double_precision: bool = False,
    **kwargs,
):
    """
    Convert a scene graph to a homogeneous pytorch geometric data structure.

    Args:
        G: scene graph to convert
        node_converter: function to generate input node features
        edge_converter: optional function to generate input edge features
        layer_name_map: optional map between layer ids and names.
        is_undirected: whether or not the graph is undirected.
        double_precision: whether or not output data attributes have double precision.
        **kwargs: absorbing keyword arguments for homogeneous conversion arguments

    Raises:
        ValueError: If pytorch geometric can't be found for the conversion

    Returns:
        pytorch_geometric.Data: homogeneous pytorch_geometric graph representing the
            scene graph.
    """
    torch, torch_geometric = _get_torch()

    # output torch tensor data types
    if double_precision:
        dtype_int = torch.int64
        dtype_float = torch.float64
    else:
        dtype_int = torch.int32
        dtype_float = torch.float32

    layer_map = DEFAULT_LAYER_MAP if layer_name_map is None else layer_name_map
    edge_map = _get_edge_name_map(layer_map)
    edge_type_map = {}
    for l1, l1_map in edge_map.items():
        for l2, l2_info in l1_map.items():
            edge_type_map[l2_info[1]] = l2_info[0], l2_info[2]

    data = torch_geometric.data.HeteroData()

    node_features = {}
    node_positions = {}
    node_labels = {}
    node_ids = {}
    id_map = {}

    for node in G.nodes:
        if node.layer not in node_features:
            node_features[node.layer] = []
            node_positions[node.layer] = []
            node_labels[node.layer] = []
            node_ids[node.layer] = []

        idx = len(node_features[node.layer])
        node_positions[node.layer].append(np.squeeze(node.attributes.position))
        node_features[node.layer].append(node_converter(G, node))
        node_labels[node.layer].append(node.attributes.semantic_label)
        node_ids[node.layer].append(node.id.value)
        id_map[node.id.value] = idx

    for layer in node_features:
        data[layer_map[layer]].x = torch.tensor(
            np.array(node_features[layer]), dtype=dtype_float
        )
        data[layer_map[layer]].pos = torch.tensor(
            np.array(node_positions[layer]), dtype=dtype_float
        )
        data[layer_map[layer]].label = torch.tensor(
            np.array(node_labels[layer]), dtype=dtype_int
        )
        id_tensor = torch.tensor(np.array(node_ids[layer]), dtype=torch.int64)
        data[layer_map[layer]].node_ids = id_tensor

    edge_indices = {}
    edge_features = {}
    for edge in G.edges:
        source = G.get_node(edge.source)
        target = G.get_node(edge.target)
        edge_type = edge_map[source.layer][target.layer][1]
        if edge_type not in edge_indices:
            edge_indices[edge_type] = []
            edge_features[edge_type] = []

        edge_indices[edge_type].append(_get_directed_edge(G, edge, id_map))
        if edge_converter is not None:
            edge_features[edge_type].append(edge_converter(G, edge))

    for edge_type in edge_indices:
        source_type, target_type = edge_type_map[edge_type]
        edge_index = torch.tensor(np.array(edge_indices[edge_type]).T)
        edge_attrs = None
        if len(edge_features[edge_type]) == edge_index.size(dim=1):
            edge_attrs = torch.tensor(
                np.array(edge_features[edge_type]), dtype=dtype_float
            )

        if edge_index.size(dim=1) > 0 and is_undirected and source_type == target_type:
            if edge_converter is None:
                edge_index = torch_geometric.utils.to_undirected(edge_index)
            else:
                edge_index, edge_attrs = torch_geometric.utils.to_undirected(
                    edge_index, edge_attrs
                )

        data[source_type, edge_type, target_type].edge_index = edge_index
        if edge_converter is not None:
            data[source_type, edge_type, target_type].edge_attrs = edge_attrs

    return data


def scene_graph_to_torch(
    G: DynamicSceneGraph, *args, use_heterogeneous: bool = True, **kwargs
):
    """
    Convert a scene graph to a pytorch geometric graph.

    Args:
        G: scene graph to convert
        *args: All positional arguments for scene_graph_to_torch_homogeneous or
               scene_graph_to_torch_heterogeneous
        use_heterogeneous: Whether or not to use a heterogeneous pytorch geometric graph
            structure
        **kwargs: All arguments for scene_graph_to_torch_homogeneous or
                  scene_graph_to_torch_heterogeneous

    Raises:
        ValueError: If pytorch geometric can't be found for the conversion

    Returns:
        Union[pytorch_geometric.HeteroData, pytorch_geometric.Data]: pytorch geometric
            data representing the scene graph depending on use_heterogeneous.
    """
    if use_heterogeneous:
        return scene_graph_to_torch_heterogeneous(G, *args, **kwargs)
    else:
        return scene_graph_to_torch_homogeneous(G, *args, **kwargs)
