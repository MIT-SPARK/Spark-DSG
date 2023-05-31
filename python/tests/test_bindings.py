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
"""Test that the bindings are working appropriately."""
import spark_dsg as dsg
import numpy as np


def test_empty_graph():
    """Test that an new graph is empty."""
    G = dsg.DynamicSceneGraph()
    assert G.num_nodes() == 0
    assert G.num_edges() == 0


def test_implicit_prefix():
    """Test that we got rid of the need for explicit layer prefix construction."""
    G = dsg.DynamicSceneGraph()
    G.create_dynamic_layer(dsg.DsgLayers.AGENTS, "a")
    assert G.has_layer(dsg.DsgLayers.AGENTS, "a")


def test_layer_ids(resource_dir):
    """Test that layer ids show up correctly."""
    mp3d_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(mp3d_dsg))

    layer_ids = [layer.id for layer in G.layers]
    assert layer_ids == [
        dsg.DsgLayers.OBJECTS,
        dsg.DsgLayers.PLACES,
        dsg.DsgLayers.ROOMS,
        dsg.DsgLayers.BUILDINGS,
    ]

    # TODO(nathan) add to bindings
    # assert G.mesh_layer_id == 1


def test_add_remove(resource_dir):
    """Test that adding and removing nodes works as expected."""
    mp3d_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(mp3d_dsg))

    G_new = dsg.DynamicSceneGraph()

    # add nodes
    for node in G.nodes:
        G_new.add_node(node.layer, node.id.value, node.attributes)

    # add edges
    for edge in G.edges:
        assert G_new.insert_edge(edge.source, edge.target, edge.info)

    assert G.num_static_nodes() == G_new.num_static_nodes()
    assert G.num_static_edges() == G_new.num_static_edges()

    # n.b. removing in-place (while iterating) creates undefined behavior
    # and will likely segfault

    # remove edges
    for edge in G.edges:
        assert G_new.remove_edge(edge.source, edge.target)

    # remove nodes
    for node in G.nodes:
        assert G_new.remove_node(node.id.value)

    assert G_new.num_nodes() == 0
    assert G_new.num_edges() == 0


def _check_parent(node, expected_category=None):
    has_parent = node.has_parent()
    assert has_parent is True or has_parent is False

    if not has_parent:
        assert node.get_parent() is None
        return

    parent = node.get_parent()
    assert parent is not None
    if expected_category is None:
        return

    assert dsg.NodeSymbol(parent).category == "p"


def _check_siblings(G, node):
    if not node.has_siblings():
        assert len(node.siblings()) == 0
        return

    for sibling in node.siblings():
        assert G.has_node(sibling)


def _check_children(G, node):
    if not node.has_children():
        assert len(node.children()) == 0
        return

    for child in node.children():
        assert G.has_node(child)


def _check_base_attributes(attrs):
    assert hasattr(attrs, "position")
    assert attrs.position.shape == (3,)
    assert hasattr(attrs, "last_update_time_ns")


def _check_semantic_attributes(attrs, node_id=None, bbox_valid=False):
    assert hasattr(attrs, "color")
    assert attrs.color.shape == (3,)
    assert attrs.color.dtype == np.uint8
    assert hasattr(attrs, "name")
    if node_id is not None:
        assert attrs.name == str(node_id)
    assert hasattr(attrs, "bounding_box")
    assert hasattr(attrs, "semantic_label")
    if bbox_valid:
        assert attrs.bounding_box.type != dsg.BoundingBoxType.INVALID


def test_agent_attributes(resource_dir):
    """Test that agent attributes work correctly."""
    mp3d_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(mp3d_dsg))

    agents = G.get_dynamic_layer(dsg.DsgLayers.AGENTS, "a")
    for agent in agents.nodes:
        assert hasattr(agent, "id")
        assert agent.id.category == "a"
        assert agent.layer == dsg.DsgLayers.AGENTS
        assert hasattr(agent, "timestamp")

        _check_parent(agent)
        _check_siblings(G, agent)
        _check_base_attributes(agent.attributes)

        assert hasattr(agent.attributes, "world_R_body")


def test_object_attributes(resource_dir):
    """Test that object attributes work correctly."""
    mp3d_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(mp3d_dsg))

    objects = G.get_layer(dsg.DsgLayers.OBJECTS)
    for node in objects.nodes:
        assert hasattr(node, "id")
        assert node.id.category == "O"
        assert node.layer == dsg.DsgLayers.OBJECTS

        _check_parent(node)
        _check_base_attributes(node.attributes)
        _check_semantic_attributes(node.attributes, node_id=node.id, bbox_valid=True)

        assert hasattr(node.attributes, "registered")
        assert hasattr(node.attributes, "world_R_object")


def test_place_attributes(resource_dir):
    """Test that place attributes work correctly."""
    mp3d_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(mp3d_dsg))

    places = G.get_layer(dsg.DsgLayers.PLACES)
    for node in places.nodes:
        assert hasattr(node, "id")
        assert node.id.category == "p"
        assert node.layer == dsg.DsgLayers.PLACES

        _check_parent(node)
        _check_siblings(G, node)
        _check_children(G, node)
        _check_base_attributes(node.attributes)
        _check_semantic_attributes(node.attributes)

        assert hasattr(node.attributes, "distance")
        assert hasattr(node.attributes, "num_basis_points")
        assert hasattr(node.attributes, "is_active")
        assert hasattr(node.attributes, "voxblox_mesh_connections")
        assert hasattr(node.attributes, "mesh_vertex_labels")
        assert hasattr(node.attributes, "deformation_connections")
        assert hasattr(node.attributes, "pcl_mesh_connections")


def test_room_attributes(resource_dir):
    """Test that room attributes work correctly."""
    mp3d_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(mp3d_dsg))

    rooms = G.get_layer(dsg.DsgLayers.ROOMS)
    for node in rooms.nodes:
        assert hasattr(node, "id")
        assert node.id.category == "R"
        assert node.layer == dsg.DsgLayers.ROOMS

        _check_parent(node)
        _check_siblings(G, node)
        _check_children(G, node)
        _check_base_attributes(node.attributes)
        _check_semantic_attributes(node.attributes)

    dsg.add_bounding_boxes_to_layer(G, dsg.DsgLayers.ROOMS)

    rooms = G.get_layer(dsg.DsgLayers.ROOMS)
    for node in rooms.nodes:
        _check_semantic_attributes(node.attributes, bbox_valid=True)


def test_building_attributes(resource_dir):
    """Test that building attributes work correctly."""
    uh2_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(uh2_dsg))

    buildings = G.get_layer(dsg.DsgLayers.BUILDINGS)
    for node in buildings.nodes:
        assert hasattr(node, "id")
        assert node.id.category == "B"
        assert node.layer == dsg.DsgLayers.BUILDINGS

        _check_parent(node)
        _check_siblings(G, node)
        _check_children(G, node)
        _check_base_attributes(node.attributes)
        _check_semantic_attributes(node.attributes)

    dsg.add_bounding_boxes_to_layer(G, dsg.DsgLayers.BUILDINGS)

    buildings = G.get_layer(dsg.DsgLayers.BUILDINGS)
    for node in buildings.nodes:
        _check_semantic_attributes(node.attributes, bbox_valid=True)


def _check_layer_edges(G, layer_id):
    layer = G.get_layer(layer_id)
    for edge in layer.edges:
        n1 = layer.get_node(edge.source)
        n2 = layer.get_node(edge.target)
        assert n2.id.value in n1.siblings()
        assert n1.id.value in n2.siblings()


def test_intralayer_edges(resource_dir):
    """Test that edges are present in layers that we expect."""
    uh2_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(uh2_dsg))

    _check_layer_edges(G, dsg.DsgLayers.PLACES)
    _check_layer_edges(G, dsg.DsgLayers.ROOMS)


def test_node_counts(resource_dir):
    """Test that nodes are present in layers that we expect."""
    uh2_dsg = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(uh2_dsg))

    node_type_counts = {}
    for node in G.nodes:
        if node.id.category not in node_type_counts:
            node_type_counts[node.id.category] = 0

        node_type_counts[node.id.category] += 1

    assert "O" in node_type_counts
    assert node_type_counts["O"] == G.get_layer(dsg.DsgLayers.OBJECTS).num_nodes()
    assert "R" in node_type_counts
    assert node_type_counts["R"] == G.get_layer(dsg.DsgLayers.ROOMS).num_nodes()
    assert "B" in node_type_counts
    assert node_type_counts["B"] == G.get_layer(dsg.DsgLayers.BUILDINGS).num_nodes()
