import importlib
import logging


def _get_networkx():
    networkx = None
    try:
        networkx = importlib.import_module("networkx")
    except ImportError:
        logging.warning("networkx not found. conversion disabled")

    return networkx


def _convert_attr(attrs):
    valid_fields = [x for x in dir(attrs) if x[0] != "_"]
    return {x: getattr(attrs, x) for x in valid_fields}


def _fill_from_layer(G_out, layer):
    for node in layer.nodes:
        G_out.add_node(node.id.value, **_convert_attr(node.attributes))

    for edge in layer.edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))


def graph_to_networkx(G_in, include_dynamic=True):
    """Convert the DSG to a networkx representation."""
    nx = _get_networkx()
    if nx is None:
        return None

    G_out = nx.Graph()
    for layer in G_in.layers:
        _fill_from_layer(G_out, layer)

    for edge in G_in.interlayer_edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))

    if not include_dynamic:
        return G_out

    for layer in G_in.dynamic_layers:
        _fill_from_layer(G_out, layer)

    for edge in G_in.dynamic_interlayer_edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))

    return G_out


def layer_to_networkx(G_in):
    """Convert the DSG to a networkx representation."""
    nx = _get_networkx()
    if nx is None:
        return None

    G_out = nx.Graph()
    _fill_from_layer(G_out, G_in)
    return G_out
