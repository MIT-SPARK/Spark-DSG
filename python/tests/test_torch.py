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
"""Test that pytorch conversion works as expected."""
import spark_dsg as dsg
import numpy as np
import pytest


def _fake_node_embedding(G, x):
    return np.zeros(20)


def _fake_edge_embedding(G, e):
    return np.zeros(20)


def _check_graph_layers(G, data, to_check, has_edge_attrs=False):
    metadata = data.metadata()
    for layer_id, name in to_check:
        layer = G.get_layer(layer_id)
        if layer.num_nodes() > 0:
            assert name in metadata[0]
            assert data[name].x.size() == (layer.num_nodes(), 20)
            assert data[name].x.size() == (layer.num_nodes(), 20)

        edge_name = f"{name}_to_{name}"
        num_undirected = 2 * layer.num_edges()
        if layer.num_edges() > 0:
            assert (name, edge_name, name) in metadata[1]
            assert data[name, edge_name, name].edge_index.size() == (2, num_undirected)
            if has_edge_attrs:
                assert data[name, edge_name, name].edge_attr.size() == (
                    num_undirected,
                    20,
                )


def _check_interlayer_edges(G, data, to_check, has_edge_attrs=False):
    metadata = data.metadata()
    for source, target in to_check:
        edge_name = f"{source}_to_{target}"
        assert (source, edge_name, target) in metadata[1]
        assert data[source, edge_name, target].edge_index.size(dim=0) == 2
        assert data[source, edge_name, target].edge_index.size(dim=1) >= 2
        if has_edge_attrs:
            assert data[source, edge_name, target].edge_attr.size(dim=1) == 20
            assert data[source, edge_name, target].edge_attr.size(dim=0) == data[
                source, edge_name, target
            ].edge_index.size(dim=1)


def test_torch_layer(resource_dir, has_torch):
    """Test that layer conversion (without edge features) works."""
    if not has_torch:
        return pytest.skip(reason="requires pytorch and pytorch geometric")

    G = dsg.DynamicSceneGraph.load(str(resource_dir / "apartment_igx_dsg.json"))
    places = G.get_layer(dsg.DsgLayers.PLACES)
    assert places.num_nodes() > 0
    assert places.num_edges() > 0

    data = places.to_torch(_fake_node_embedding)

    assert data.x.size() == (places.num_nodes(), 20)
    assert data.pos.size() == (places.num_nodes(), 3)
    assert data.edge_index.size() == (2, 2 * places.num_edges())
    assert data.edge_attr is None


def test_torch_layer_edge_features(resource_dir, has_torch):
    """Test that layer conversion (with edge features) works."""
    if not has_torch:
        return pytest.skip(reason="requires pytorch and pytorch geometric")

    G = dsg.DynamicSceneGraph.load(str(resource_dir / "apartment_igx_dsg.json"))
    places = G.get_layer(dsg.DsgLayers.PLACES)
    assert places.num_nodes() > 0
    assert places.num_edges() > 0

    data = places.to_torch(_fake_node_embedding, _fake_edge_embedding)

    assert data.x.size() == (places.num_nodes(), 20)
    assert data.pos.size() == (places.num_nodes(), 3)
    assert data.edge_index.size() == (2, 2 * places.num_edges())
    assert data.edge_attr.size() == (2 * places.num_edges(), 20)


def test_torch_homogeneous(resource_dir, has_torch):
    """Test that homogeneous conversion (without edge features) works."""
    if not has_torch:
        return pytest.skip(reason="requires pytorch and pytorch geometric")

    G = dsg.DynamicSceneGraph.load(str(resource_dir / "apartment_igx_dsg.json"))
    assert G.num_nodes() > 0
    assert G.num_edges() > 0

    data = G.to_torch(_fake_node_embedding, use_heterogeneous=False)

    assert data.x.size() == (G.num_static_nodes(), 20)
    assert data.pos.size() == (G.num_static_nodes(), 3)
    assert data.edge_index.size() == (2, 2 * G.num_static_edges())
    assert data.edge_attr is None


def test_torch_homogeneous_edge_features(resource_dir, has_torch):
    """Test that homogeneous conversion (with edge features) works."""
    if not has_torch:
        return pytest.skip(reason="requires pytorch and pytorch geometric")

    G = dsg.DynamicSceneGraph.load(str(resource_dir / "apartment_igx_dsg.json"))
    assert G.num_nodes() > 0
    assert G.num_edges() > 0

    data = G.to_torch(
        _fake_node_embedding,
        use_heterogeneous=False,
        edge_converter=_fake_edge_embedding,
    )

    assert data.x.size() == (G.num_static_nodes(), 20)
    assert data.pos.size() == (G.num_static_nodes(), 3)
    assert data.edge_index.size() == (2, 2 * G.num_static_edges())
    assert data.edge_attr.size() == (2 * G.num_static_edges(), 20)


def test_torch_hetereogeneous(resource_dir, has_torch):
    """Test that heterogeneous conversion (without edge features) works."""
    if not has_torch:
        return pytest.skip(reason="requires pytorch and pytorch geometric")

    G = dsg.DynamicSceneGraph.load(str(resource_dir / "apartment_igx_dsg.json"))
    assert G.num_nodes() > 0
    assert G.num_edges() > 0

    data = G.to_torch(_fake_node_embedding, use_heterogeneous=True)
    node_types, edge_types = data.metadata()

    # check that all the node and edge types that should exist do exist
    to_check = [
        (dsg.DsgLayers.OBJECTS, "objects"),
        (dsg.DsgLayers.PLACES, "places"),
        (dsg.DsgLayers.ROOMS, "rooms"),
        (dsg.DsgLayers.BUILDINGS, "buildings"),
    ]

    _check_graph_layers(G, data, to_check)

    to_check_interlayer = [
        ("places", "objects"),
        ("rooms", "places"),
        ("buildings", "rooms"),
    ]
    _check_interlayer_edges(G, data, to_check_interlayer)


def test_torch_hetereogeneous_edge_features(resource_dir, has_torch):
    """Test that heterogeneous conversion (with edge features) works."""
    if not has_torch:
        return pytest.skip(reason="requires pytorch and pytorch geometric")

    G = dsg.DynamicSceneGraph.load(str(resource_dir / "apartment_igx_dsg.json"))
    assert G.num_nodes() > 0
    assert G.num_edges() > 0

    data = G.to_torch(
        _fake_node_embedding,
        use_heterogeneous=True,
        edge_converter=_fake_edge_embedding,
    )

    # check that all the node and edge types that should exist do exist
    to_check = [
        (dsg.DsgLayers.OBJECTS, "objects"),
        (dsg.DsgLayers.PLACES, "places"),
        (dsg.DsgLayers.ROOMS, "rooms"),
        (dsg.DsgLayers.BUILDINGS, "buildings"),
    ]
    _check_graph_layers(G, data, to_check)

    to_check_interlayer = [
        ("places", "objects"),
        ("rooms", "places"),
        ("buildings", "rooms"),
    ]
    _check_interlayer_edges(G, data, to_check_interlayer)
