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
