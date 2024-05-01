# %%
from spark_dsg._dsg_bindings import DsgLayers, NodeSymbol, DynamicSceneGraph
from spark_dsg.visualization import plot_scene_graph 
import pathlib
import numpy as np

# %%
path_to_dsg = "~/put/backend/dsg.json"
path_to_dsg = pathlib.Path(path_to_dsg).expanduser().absolute()

# %%
G = DynamicSceneGraph.load(str(path_to_dsg))

# %%
fig = plot_scene_graph(G)
if fig is not None:
    fig.show(renderer="notebook")

# %%
text = {"nodes": {"room" : [], 
                "agent": [{"loc" : None, "id": "agent"}]}, 
        "links": []}

rl = G.get_layer(DsgLayers.ROOMS)

agent_location = list(rl.nodes)[0]
text["nodes"]["agent"][0]["loc"] = agent_location.id

for room_node in rl.nodes:
    room_id = room_node.id
    text["nodes"]["room"].append({"id": room_id})


for room_edge in rl.edges:
    source = G.get_node(room_edge.source)
    target = G.get_node(room_edge.target)
    text["links"].append(f"{source.id}<->{target.id}")

print(text)

# %%

def expand(G, room_id, text):
    room = G.get_node(room_id)
    if "asset" not in text["nodes"]:
        text["nodes"]["asset"] = []
    for place_id in room.children():
        place_node = G.get_node(place_id)
        for object_id in place_node.children():
            object_node = G.get_node(object_id)
            if object_node.id.category == 'a':
                continue
            position = list(np.round(object_node.attributes.position, 3))
            label = object_node.attributes.semantic_label
            asset = {"room": room.id, "label": label, "loc": position, "id": object_node.id}
            text["nodes"]["asset"].append(asset)
    return text

def contract(G, room_id, text):
    room = G.get_node(room_id)
    assets = []
    if "asset" not in text["nodes"]:
        return

    for asset in text["nodes"]["asset"]:
        if str(asset['room']) != str(room.id):
            assets.append(asset)

    text["nodes"]["asset"] = assets
    return text

# %%

# text = expand(G, list(rl.nodes)[0].id.value, sg)
text = expand(G, list(rl.nodes)[1].id.value, text)
text

# %%

text = contract(G, list(rl.nodes)[1].id.value, text)
text

# %%

text = expand(G, list(rl.nodes)[0].id.value, text)
text

# %%

text = contract(G, list(rl.nodes)[0].id.value, text)
text