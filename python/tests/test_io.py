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
    graphs = {
        i: _make_layer(dsg.DsgLayers.OBJECTS, end=5 + i)
        for i in range(4)
    }
    output_path = tmp_path / "test_layers"
    dsg_io.LayerCollection.save(graphs, output_path)
    assert output_path.with_suffix(".json").exists()
    assert output_path.with_suffix(".graphs").exists()


def test_load_collection(tmp_path):
    """Test that we can load graphs correctly."""
    graphs = {
        i: _make_layer(dsg.DsgLayers.OBJECTS, end=5 + i)
        for i in range(4)
    }
    output_path = tmp_path / "test_layers"
    dsg_io.LayerCollection.save(graphs, output_path)

    with dsg_io.LayerCollection(output_path) as collection:
        assert len(collection) == 4
        for i in range(4):
            assert collection[i].num_nodes() == 5 + i

        for index, graph in enumerate(collection):
            assert graph.num_nodes() == 5 + index

        assert set(collection.indices) == set(range(4))
