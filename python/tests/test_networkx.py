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
"""Test that networkx conversion works as expected."""
import spark_dsg as dsg
import spark_dsg.networkx as dsg_nx
import numpy as np


def _check_attribute_validity(G_nx):
    for node in G_nx:
        assert len(G_nx.nodes[node]) > 0
        assert "position" in G_nx.nodes[node]

    for edge in G_nx.edges:
        assert len(G_nx[edge[0]][edge[1]]) > 0
        assert "weight" in G_nx[edge[0]][edge[1]]
        assert "weighted" in G_nx[edge[0]][edge[1]]


def test_static_graph_conversion(resource_dir):
    """Test that graph conversion is exact."""
    dsg_path = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(dsg_path))
    G_nx = dsg_nx.graph_to_networkx(G, include_dynamic=False)

    assert G_nx is not None
    assert len(G_nx) == G.num_static_nodes()
    assert len(G_nx.edges) == G.num_static_edges()

    _check_attribute_validity(G_nx)


def test_full_graph_conversion(resource_dir):
    """Test that graph conversion is exact."""
    dsg_path = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(dsg_path))

    # check that we have edges between static and dynamic layers
    agents = G.get_dynamic_layer(dsg.DsgLayers.AGENTS, "a")
    has_parents = np.array([x.has_parent() for x in agents.nodes])
    assert has_parents.any()

    G_nx = dsg_nx.graph_to_networkx(G, include_dynamic=True)

    assert G_nx is not None
    assert len(G_nx) == G.num_nodes(include_mesh=False)
    assert len(G_nx.edges) == G.num_edges(include_mesh=False)

    _check_attribute_validity(G_nx)


def test_layer_conversion(resource_dir):
    """Test that layer conversion is exact."""
    dsg_path = resource_dir / "apartment_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(dsg_path))
    places = G.get_layer(dsg.DsgLayers.PLACES)
    G_nx = dsg_nx.layer_to_networkx(places)

    assert G_nx is not None
    assert len(G_nx) == places.num_nodes()
    assert len(G_nx.edges) == places.num_edges()

    _check_attribute_validity(G_nx)
