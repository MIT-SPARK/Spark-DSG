#!/usr/bin/env python
"""Quick test of scene graph bindings."""
import kimera_dsg_python_bindings as kimera_dsg


def main():
    """Demo some bindings."""
    output = "/home/ubuntu/catkin_ws/src/kimera_scene_graph/kimera_scene_graph/output/"
    filename = "goseek_scene7_dsg.json"

    graph = kimera_dsg.SceneGraph()
    graph.load(output + filename)
    print(
        "Scene Graph: {} nodes, {} edges".format(graph.num_nodes(), graph.num_edges())
    )
    print("")

    test_symbol = kimera_dsg.NodeSymbol("O", 0)
    print("Checking for {}:".format(test_symbol))
    print(
        "  - graph has node? {}".format(
            "yes" if graph.has_node(test_symbol.value) else "no"
        )
    )
    if graph.has_node(test_symbol.value):
        node = graph.get_node(test_symbol.value)
        print(" - {}".format(node.attributes))

    print("Layers:")
    for layer in graph.layers:
        print("  - {}".format(layer.id))

    print("Interlayer Edges:")
    layer_edge_counts = {}
    for edge in graph.inter_layer_edges:
        source_layer = graph.get_node(edge.source).layer
        target_layer = graph.get_node(edge.target).layer
        if source_layer not in layer_edge_counts:
            layer_edge_counts[source_layer] = {}
        if target_layer not in layer_edge_counts[source_layer]:
            layer_edge_counts[source_layer][target_layer] = 0

        layer_edge_counts[source_layer][target_layer] += 1

    for source_layer in layer_edge_counts:
        print("  - {} -> {}".format(source_layer, layer_edge_counts[source_layer]))

    room_layer = graph.get_layer(kimera_dsg.KimeraDsgLayers.ROOMS)
    print("Rooms:")
    for node in room_layer.nodes:
        print("  - {}".format(node))
    print("Room Edges:")
    for edge in room_layer.edges:
        print("  - {}".format(edge))


if __name__ == "__main__":
    main()
