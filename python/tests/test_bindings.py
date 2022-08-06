import spark_dsg as dsg


def test_empty_graph():
    """Test that an new graph is empty."""
    G = dsg.DynamicSceneGraph()
    assert G.num_nodes() == 0
    assert G.num_edges() == 0
