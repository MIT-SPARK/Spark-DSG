# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.
#
#
# -*- coding: utf-8 -*-
# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.13.4
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# %%
import spark_dsg as dsg
import pathlib


# %%
path_to_dsg = "~/uh2_office_dsg.json"
path_to_dsg = pathlib.Path(path_to_dsg).expanduser().absolute()


# %%
G = dsg.DynamicSceneGraph.load(str(path_to_dsg))


# %%
fig = dsg.plot_scene_graph(G, marker_size=6)
if fig is not None:
    fig.show(renderer="notebook")


# %% [markdown]
# ## Node Access and Node Attributes


# %%
def print_node_if_exists(node_id):
    """Grab node attributes and display if the graph contains the node."""
    test_str = "yes" if G.has_node(node_id.value) else "no"
    print(f"Graph contains node {node_id}: {test_str}")

    if not G.has_node(node_id.value):
        return

    node = G.get_node(node_id.value)
    print(f"Node {node_id} attributes:\n{node.attributes}")


# object node
print("")
print_node_if_exists(dsg.NodeSymbol("O", 4))

# places node
print("")
print_node_if_exists(dsg.NodeSymbol("p", 135))

# room node
print("")
print_node_if_exists(dsg.NodeSymbol("R", 19))


# agent node
print("")
print_node_if_exists(dsg.NodeSymbol("a", 5))


# %% [markdown]
# ## Layer Iteration and Access

# %%
print("Layers:")
for layer in G.layers:
    print(f"  - {layer.id}")
print("")

# %%
room_layer = G.get_layer(dsg.DsgLayers.ROOMS)
room_node_strs = [f"{x.id}" for x in room_layer.nodes]
print(f"Rooms: {room_node_strs}")

room_edge_strs = [f"  - {x}" for x in room_layer.edges]
print("Room Edges:")
print("\n".join(room_edge_strs))
print("")


# %% [markdown]
# ## Interlayer Edge Access

# %%
layer_edge_counts = {}
for edge in G.interlayer_edges:
    source_layer = G.get_node(edge.source).layer
    target_layer = G.get_node(edge.target).layer
    if source_layer not in layer_edge_counts:
        layer_edge_counts[source_layer] = {}
    if target_layer not in layer_edge_counts[source_layer]:
        layer_edge_counts[source_layer][target_layer] = 0

    layer_edge_counts[source_layer][target_layer] += 1

print("Interlayer Edges:")
for source_layer in layer_edge_counts:
    print(f"  - {source_layer} -> {layer_edge_counts[source_layer]}")
print("")


# %% [markdown]
# ## Entire Graph Access

# %%
node_type_counts = {}
for node in G.nodes:
    if node.id.category not in node_type_counts:
        node_type_counts[node.id.category] = 0

    node_type_counts[node.id.category] += 1

print("Node Types:")
for category, count in node_type_counts.items():
    print(f"  - {category}: {count}")


edge_counts = {}
for edge in G.edges:
    source_layer = G.get_node(edge.source).layer
    target_layer = G.get_node(edge.target).layer
    if source_layer not in edge_counts:
        edge_counts[source_layer] = {}
    if target_layer not in edge_counts[source_layer]:
        edge_counts[source_layer][target_layer] = 0

    edge_counts[source_layer][target_layer] += 1

print("All Edges:")
for source_layer in edge_counts:
    print(f"  - {source_layer} -> {edge_counts[source_layer]}")
print("")


# %% [markdown]
# ## Bounding boxes from nodes in lower layers

# %%
dsg.add_bounding_boxes_to_layer(G, dsg.DsgLayers.ROOMS)
print("Room bounding boxes:")
for node in G.get_layer(dsg.DsgLayers.ROOMS).nodes:
    print(f"  - {node.id}: {node.attributes.bounding_box}")
