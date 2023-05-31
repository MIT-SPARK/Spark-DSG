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
"""Test that collections of graphs work correctly."""
import spark_dsg as dsg
import spark_dsg.io as dsg_io
import numpy as np


def _make_layer(layer, end, start=0):
    G = dsg.SceneGraphLayer(layer)
    for index in range(start, end):
        if layer == dsg.DsgLayers.OBJECTS:
            attrs = dsg.ObjectNodeAttributes()
        elif layer == dsg.DsgLayers.PLACES:
            attrs = dsg.PlaceNodeAttributes()
        else:
            attrs = dsg.NodeAttributes()
        attrs.position = np.array([index, 0, 0])
        G.add_node(index, attrs)

    for index in range(start + 1, end):
        info = dsg.EdgeAttributes()
        info.weight = index
        info.weighted = True
        G.insert_edge(index - 1, index, info)

    return G


def test_save_collection(tmp_path):
    """Test that we can save graphs correctly."""
    graphs = {i: _make_layer(dsg.DsgLayers.OBJECTS, end=5 + i) for i in range(4)}
    output_path = tmp_path / "test_layers"
    dsg_io.LayerCollection.save(graphs, output_path)
    assert output_path.with_suffix(".json").exists()
    assert output_path.with_suffix(".graphs").exists()


def test_load_collection(tmp_path):
    """Test that we can load graphs correctly."""
    graphs = {i: _make_layer(dsg.DsgLayers.OBJECTS, end=5 + i) for i in range(4)}
    output_path = tmp_path / "test_layers"
    dsg_io.LayerCollection.save(graphs, output_path)

    with dsg_io.LayerCollection(output_path) as collection:
        assert len(collection) == 4
        for i in range(4):
            assert collection[i].num_nodes() == 5 + i

        for index, graph in enumerate(collection):
            assert graph.num_nodes() == 5 + index

        assert set(collection.indices) == set(range(4))
