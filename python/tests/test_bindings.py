"""Test that the bindings are working appropriately."""
import spark_dsg as dsg


def test_empty_graph():
    """Test that an new graph is empty."""
    G = dsg.DynamicSceneGraph()
    assert G.num_nodes() == 0
    assert G.num_edges() == 0


def test_implicit_prefix():
    """Test that we got rid of the need for explicit layer prefix construction."""
    G = dsg.DynamicSceneGraph()
    G.create_dynamic_layer(dsg.DsgLayers.AGENTS, "a")
    assert G.has_layer(dsg.DsgLayers.AGENTS, "a")
