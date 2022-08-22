"""Test that networkx conversion works as expected."""
import spark_dsg as dsg
import spark_dsg.networkx as dsg_nx


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
    dsg_path = resource_dir / "apartment_igx_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(dsg_path))
    G_nx = dsg_nx.graph_to_networkx(G, include_dynamic=False)

    assert G_nx is not None
    assert len(G_nx) == G.num_static_nodes()
    assert len(G_nx.edges) == G.num_static_edges()

    _check_attribute_validity(G_nx)


def test_full_graph_conversion(resource_dir):
    """Test that graph conversion is exact."""
    dsg_path = resource_dir / "apartment_igx_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(dsg_path))

    # add a dynamic interlayer edge because the graph doesn't contain them
    agents = G.get_dynamic_layer(dsg.DsgLayers.AGENTS, "a")
    first_agent = next(agents.nodes).id.value
    places = G.get_layer(dsg.DsgLayers.PLACES)
    first_place = next(places.nodes).id.value
    assert G.insert_edge(first_agent, first_place)

    G_nx = dsg_nx.graph_to_networkx(G, include_dynamic=True)

    assert G_nx is not None
    assert len(G_nx) == G.num_nodes(include_mesh=False)
    assert len(G_nx.edges) == G.num_edges(include_mesh=False)

    _check_attribute_validity(G_nx)


def test_layer_conversion(resource_dir):
    """Test that layer conversion is exact."""
    dsg_path = resource_dir / "apartment_igx_dsg.json"
    G = dsg.DynamicSceneGraph.load(str(dsg_path))
    places = G.get_layer(dsg.DsgLayers.PLACES)
    G_nx = dsg_nx.layer_to_networkx(places)

    assert G_nx is not None
    assert len(G_nx) == places.num_nodes()
    assert len(G_nx.edges) == places.num_edges()

    _check_attribute_validity(G_nx)
