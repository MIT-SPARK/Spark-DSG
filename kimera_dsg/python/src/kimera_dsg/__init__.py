"""The Kimera-DSG package."""
from kimera_dsg._dsg_bindings import *


def add_bounding_boxes_to_layer(graph, layer_id):
    """
    Add computed bounding boxes to the node attributes in the graph.

    Computes the bounding box from the centroids of the children. The computed
    bounding box is axis-aligned unless it is empty (in which case it is invalid).

    Args:
        graph (kimera_dsg._dsg_bindings.DynamicSceneGraph): scene graph
        layer_id (int): layer to add bindings to
    """
    layer = graph.get_layer(layer_id)
    for node in layer.nodes:
        bbox = compute_ancestor_bounding_box(graph, node.id.value, KimeraDsgLayers.PLACES)
        node.attributes.bounding_box = bbox
