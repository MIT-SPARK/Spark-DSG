import spark_dsg.mp3d as dsg_mp3d
import spark_dsg as dsg


def test_load_mp3d_info(resource_dir):
    house_file = resource_dir / "example_mp3d_scene.house"
    info = dsg_mp3d.load_mp3d_info(house_file)
    assert len(info["R"]) == 10
    assert len(info["S"]) == 10
    assert len(info["V"]) == 57
    assert len(info["C"]) == 1659
    assert len(info["O"]) == 187
    assert len(info["E"]) == 6


def test_load_mp3d_rooms(resource_dir):
    house_file = resource_dir / "example_mp3d_scene.house"
    info = dsg_mp3d.load_mp3d_info(house_file)
    rooms = dsg_mp3d.get_rooms_from_mp3d_info(info)

    assert len(info["R"]) == len(rooms)


def _add_weighted_edge(G, n1, n2, weight):
    info = dsg.EdgeAttributes()
    info.weight = weight
    G.insert_edge(n1, n2, info)


def test_expand_rooms():
    G = dsg.DynamicSceneGraph()
    G.add_node(dsg.DsgLayers.ROOMS, 0, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.ROOMS, 1, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 2, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 3, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 4, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 5, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 6, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 7, dsg.NodeAttributes())
    G.add_node(dsg.DsgLayers.PLACES, 8, dsg.NodeAttributes())
    # start floodfill from 3 and 5
    G.insert_edge(0, 3)
    G.insert_edge(1, 5)
    # make a chain graph
    _add_weighted_edge(G, 2, 3, 0.1)
    _add_weighted_edge(G, 3, 4, 0.2)
    _add_weighted_edge(G, 4, 5, 0.3)
    _add_weighted_edge(G, 5, 6, 0.4)
    # add another neighbor to 3 and 5 that priortizes 3
    _add_weighted_edge(G, 3, 8, 0.3)
    _add_weighted_edge(G, 5, 8, 0.2)

    G_new = dsg_mp3d.expand_rooms(G)

    assert G_new.get_node(2).get_parent() == 0
    assert G_new.get_node(3).get_parent() == 0
    assert G_new.get_node(4).get_parent() == 1
    assert G_new.get_node(5).get_parent() == 1
    assert G_new.get_node(6).get_parent() == 1
    assert G_new.get_node(7).get_parent() is None
    assert G_new.get_node(8).get_parent() == 0
