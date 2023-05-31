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
"""Test mp3d utilities."""
import spark_dsg.mp3d as dsg_mp3d
import spark_dsg as dsg


def test_load_mp3d_info(resource_dir):
    """Test that we can parse a house file correctly."""
    house_file = resource_dir / "example_mp3d_scene.house"
    info = dsg_mp3d.load_mp3d_info(house_file)
    assert len(info["R"]) == 10
    assert len(info["S"]) == 10
    assert len(info["V"]) == 57
    assert len(info["C"]) == 1659
    assert len(info["O"]) == 187
    assert len(info["E"]) == 6


def test_load_mp3d_rooms(resource_dir):
    """Test that we can parse rooms from a house file."""
    house_file = resource_dir / "example_mp3d_scene.house"
    info = dsg_mp3d.load_mp3d_info(house_file)
    rooms = dsg_mp3d.get_rooms_from_mp3d_info(info)

    assert len(info["R"]) == len(rooms)


def _add_weighted_edge(G, n1, n2, weight):
    info = dsg.EdgeAttributes()
    info.weight = weight
    G.insert_edge(n1, n2, info)


def test_expand_rooms():
    """Test that we correctly pad out places for a given room segmentation."""
    G = dsg.DynamicSceneGraph()
    G.add_node(dsg.DsgLayers.ROOMS, 0, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.ROOMS, 1, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 2, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 3, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 4, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 5, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 6, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 7, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 8, dsg.NodeAttributes())
    # start floodfill from 3 and 5
    G.insert_edge(0, 3)
    G.insert_edge(1, 5)
    # make a chain graph
    _add_weighted_edge(G, 2, 3, 0.1)
    _add_weighted_edge(G, 3, 4, 0.2)
    _add_weighted_edge(G, 4, 5, 0.3)
    _add_weighted_edge(G, 5, 6, 0.4)
    # add another neighbor to 3 and 5 that priortizes 3
    _add_weighted_edge(G, 3, 8, 0.3)
    _add_weighted_edge(G, 5, 8, 0.2)

    G_new = dsg_mp3d.expand_rooms(G)

    assert G_new.get_node(2).get_parent() == 0
    assert G_new.get_node(3).get_parent() == 0
    assert G_new.get_node(4).get_parent() == 1
    assert G_new.get_node(5).get_parent() == 1
    assert G_new.get_node(6).get_parent() == 1
    assert G_new.get_node(7).get_parent() is None
    assert G_new.get_node(8).get_parent() == 0
