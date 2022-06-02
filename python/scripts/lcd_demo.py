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
#!/usr/bin/env python
"""Quick test of scene graph bindings."""
import kimera_dsg_python_bindings as kimera_dsg
import kimera_dsg_lcd as lcd
import numpy as np


def main():
    """Demo some bindings."""
    output = "/home/user/Downloads/"
    filename = "goseek_scene7_dsg.json"

    graph = kimera_dsg.SceneGraph()
    graph.load(output + filename)
    print(
        "Scene Graph: {} nodes, {} edges".format(graph.num_nodes(), graph.num_edges())
    )

    """Transform graph to adj matrix and set of labels"""
    """ TODO: add capability to create adj matrix in API"""
    lcd_layer = graph.get_layer(kimera_dsg.KimeraDsgLayers.ROOMS)

    node_id_mapping = {}
    labels = []
    for node in lcd_layer.nodes:
        node_id_mapping[node.id] = len(labels)
        labels.append(node.attributes.semantic_label)

    adj_matrix = np.zeros([len(labels), len(labels)])
    for edge in lcd_layer.edges:
        adj_matrix[node_id_mapping[edge.source], node_id_mapping[edge.target]] = 1
        adj_matrix[node_id_mapping[edge.target], node_id_mapping[edge.source]] = 1
    print(adj_matrix)

    lcd_graph = lcd.Graph(adj_matrix, labels)
    lcd_database = lcd.Database()
    loop_closures = []
    for i in range(len(labels)):
        descp = lcd.RwDescriptor(4, 3)
        if descp.generate(lcd_graph, i):
            matches, score = lcd_database.query(descp)
            for m in matches:
                loop_closures.append((i, m))
            lcd_database.insert(i, descp)
    print("{} loop closures detected. ".format(len(matches)))

if __name__ == "__main__":
    main()
