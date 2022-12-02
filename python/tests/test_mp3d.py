import spark_dsg.mp3d as dsg_mp3d


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
