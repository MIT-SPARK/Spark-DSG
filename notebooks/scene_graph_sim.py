# %%
from spark_dsg._dsg_bindings import DsgLayers, NodeSymbol, DynamicSceneGraph
from spark_dsg import mp3d
from spark_dsg import visualization
import pathlib
import numpy as np
import json
import yaml


# %%
def _get_label_dict(labels, synonyms=None):
    """
    Get mapping from (Hydra) labels to integer label index while grouping synonym labels.
    """

    if synonyms is None or len(synonyms) == 0:
        return dict(zip(labels, range(len(labels))))

    all_labels_to_combine = [l for syn in synonyms for l in syn]
    num_labels = len(labels) - len(all_labels_to_combine) + len(synonyms)

    # label to index mapping - unique labels
    label_dict = dict(
        zip(
            [l for l in labels if l not in all_labels_to_combine]
            + [syn[0] for syn in synonyms],
            range(num_labels),
        )
    )

    # label to index mapping - synonym labels
    label_index_offset = len(labels) - len(all_labels_to_combine)
    for i, syn in enumerate(synonyms):
        for l in syn:
            label_dict[l] = i + label_index_offset

    return label_dict


class RoomLabelConverter:
    """Converter between numeric labels and actual room labels."""

    def __init__(self, name_map, synonyms=None, default="unknown"):
        """Initialize relevant maps."""
        if synonyms is None:
            self.synonyms = [("a", "t"), ("z", "Z", "x", "p", "\x15")]
        else:
            self.synonyms = synonyms

        self.default = default
        self.char_map = _get_label_dict([x for x in name_map], self.synonyms)
        self.label_map = {idx: name_map[name] for name, idx in self.char_map.items()}

    def name_from_label(self, label):
        """Get name from label."""
        return self.label_map.get(label, self.default)

    def name_from_char(self, char):
        """Get name from character."""
        label = self.char_map.get(char, self.default)
        return self.name_from_label(label)

    def __str__(self):
        """Get fancy string representation of conversion."""
        map_str = "Label Correspondence:\n"
        for idx, name in self.label_map.items():
            map_str += f"  - {idx} →  {name}\n"

        return map_str


ROOM_NAME_MAP = {
    "a": "bathroom",
    "b": "bedroom",
    "c": "closet",
    "d": "dining room",
    "e": "lobby",
    "f": "family room",
    "g": "garage",
    "h": "hallway",
    "i": "library",
    "j": "laundry room",
    "k": "kitchen",
    "l": "living room",
    "m": "conference room",
    "n": "lounge",
    "o": "office",
    "p": "porch",
    "r": "game room",
    "s": "stairwell",
    "t": "toilet",
    "u": "utility room",
    "v": "theater",
    "w": "gym",
    "x": "outdoor",
    "y": "balcony",
    "z": "other room",
    "B": "bar",
    "C": "classroom",
    "D": "dining booth",
    "S": "spa",
    "Z": "junk",
    "\x15": "unknown",
}


# %%
path_to_dsg = "~/put/backend/dsg.json"
path_to_house = "~/datasets/mp3d/1LXtFkjw3qL/1LXtFkjw3qL.house"
path_to_obj_labels = "~/catkin_ws/src/hydra/config/label_spaces/mpcat40_label_space.yaml"

# %%
class SceneGraphSimulator:

    def __init__(self, dsg_path: str, house_path: str, obj_labels_path: str):

        dsg_path = pathlib.Path(dsg_path).expanduser().absolute()
        self.G = DynamicSceneGraph.load(str(dsg_path))
        # print(len(list(self.G.get_layer(DsgLayers.ROOMS).nodes)))

        house_path = pathlib.Path(house_path).expanduser().absolute()
        mp3d_info = mp3d.load_mp3d_info(house_path)

        self.G = mp3d.repartition_rooms(self.G, mp3d_info)
        # mp3d.add_gt_room_label(self.G, mp3d_info)
        # print(len(list(self.G.get_layer(DsgLayers.ROOMS).nodes)))

        path = pathlib.Path(obj_labels_path).expanduser().absolute()
        parsed_dict = yaml.safe_load(path.read_text())
        self.obj_label_map = {item['label']: item['name'] for item in parsed_dict['label_names']}
        
        self.converter = RoomLabelConverter(ROOM_NAME_MAP)

        self.dict_rep = None
        self.collapse()
        self.add_agent(6)


    def add_agent(self, cat_id):
        room_id = NodeSymbol("R", int(cat_id))
        room_node = self.G.get_node(room_id.value)
        self.dict_rep["nodes"]["agent"] = [{"location" : self.get_room_id(room_node), "id": "agent"}]


    def get_room_id(self, room_node):
        """
        convert room node to id
        """
        semantic_label = self.converter.name_from_char(chr(room_node.attributes.semantic_label))
        return f"{semantic_label}_{room_node.id.category_id}"
    
    def get_object_id(self, object_node):
        """
        convert object node to id
        """
        semantic_label = self.obj_label_map[object_node.attributes.semantic_label]
        return f"{semantic_label}_{object_node.id.category_id}"
    
    def get_room_node(self, room_id):
        """
        convert id to room node
        """
        cat_id = room_id.split("_")[-1]
        return self.G.get_node(NodeSymbol("R", int(cat_id)).value)

    def print_rep(self):
        json_text = json.dumps(self.dict_rep)
        return json.dumps(json.loads(json_text), indent=1).replace('"', '')


    def collapse(self):

        if self.dict_rep is not None:
            del self.dict_rep["nodes"]["asset"]
            return

        self.dict_rep = {"nodes": {"room" : []}, "links": []}

        for room_node in self.G.get_layer(DsgLayers.ROOMS).nodes:
            self.dict_rep["nodes"]["room"].append({"id": self.get_room_id(room_node)})

        for room_edge in self.G.get_layer(DsgLayers.ROOMS).edges:
            source = self.G.get_node(room_edge.source)
            target = self.G.get_node(room_edge.target)
            self.dict_rep["links"].append(f"{self.get_room_id(source)}<->{self.get_room_id(target)}")


    def expand(self, room_id):

        if "asset" not in self.dict_rep["nodes"]:
            self.dict_rep["nodes"]["asset"] = []

        room_node = self.get_room_node(room_id)

        for place_id in room_node.children():
            place_node = self.G.get_node(place_id)
            for object_id in place_node.children():
                object_node = self.G.get_node(object_id)
                if object_node.id.category == 'a':
                    continue
                # position = list(np.round(object_node.attributes.position, 3))
                asset = {"room": self.get_room_id(room_node), "id": self.get_object_id(object_node)}
                self.dict_rep["nodes"]["asset"].append(asset)


    def contract(self, room_id):
        room_node = self.get_room_node(room_id)
        assets = []
        if "asset" not in self.dict_rep["nodes"]:
            return

        for asset in self.dict_rep["nodes"]["asset"]:
            if str(self.get_room_node(asset["room"]).id) != str(room_node.id):
                assets.append(asset)

        if assets == []:
            del self.dict_rep["nodes"]["asset"]
            return
        self.dict_rep["nodes"]["asset"] = assets
        return self.dict_rep


# %%
sim = SceneGraphSimulator(path_to_dsg, path_to_house, path_to_obj_labels)

# %%

roomid = "kitchen_6"
# roomid = "gym_29"
# roomid = "bedroom_30"
# roomid = "hallway_25"
# roomid = "stairwell_8"
# roomid = "unknown_27"

print(sim.print_rep())
sim.expand(roomid)
print(sim.print_rep())
sim.contract(roomid)
print(sim.print_rep())

# %%
