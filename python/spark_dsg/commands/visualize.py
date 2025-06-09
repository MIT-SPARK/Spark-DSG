"""Entry point for visualizing scene graph."""

import enum
import functools
import itertools
import time
from dataclasses import dataclass

import click
import numpy as np
import spark_dsg as dsg
import trimesh
import viser


def _layer_name(layer_key):
    return f"layer_{layer_key.layer}p{layer_key.partition}"


def color_from_label(G, node, default=None):
    if not isinstance(node.attributes, dsg.SemanticNodeAttributes):
        return default or dsg.Color()

    return dsg.distinct_150_color(node.attributes.semantic_label)


def color_from_id(G, node):
    return dsg.colorbrewer_color(node.id.category_index())


def color_from_parent(G, node, parent_func, default=None):
    if not node.has_parent:
        return default or dsg.Color()

    return parent_func(G, G.get_node(node.get_parent()))


def color_from_layer(G, node):
    return dsg.rainbow_color(node.layer.layer)


class ColorMode(enum.Enum):
    LAYER = "layer"
    ID = "id"
    LABEL = "label"
    PARENT = "parent"


def colormap_from_modes(key_to_mode, default_colors=None):
    colormap = {}
    for layer_key, mode in key_to_mode.items():
        default = default_colors.get(layer_key) if default_colors is not None else None
        if mode == ColorMode.ID:
            colormap[layer_key] = color_from_id
        elif mode == ColorMode.LABEL:
            colormap[layer_key] = functools.partial(color_from_label, default=default)
        elif mode == ColorMode.PARENT:
            colormap[layer_key] = functools.partial(
                color_from_parent, lambda G, x: colormap[x.layer](G, x), default=default
            )
        else:
            colormap[layer_key] = color_from_layer

    return colormap


class FlatGraphView:
    def __init__(self, G):
        self._pos = {}
        self._lookup = {}
        self._edges = {}
        self._interlayer_edges = {}

        for layer in itertools.chain(G.layers, G.layer_partitions):
            if layer.num_nodes() == 0:
                continue

            pos = np.zeros((layer.num_nodes(), 3))
            for idx, node in enumerate(layer.nodes):
                pos[idx, :] = node.attributes.position
                self._lookup[node.id.value] = (idx, layer.key)

            edge_tensor = np.zeros((layer.num_edges(), 2), dtype=np.int64)
            for idx, edge in enumerate(layer.edges):
                edge_tensor[idx, 0] = self._lookup[edge.source][0]
                edge_tensor[idx, 1] = self._lookup[edge.target][0]

            self._pos[layer.key] = pos
            self._edges[layer.key] = edge_tensor

        for edge in G.interlayer_edges:
            source_idx, source_layer = self._lookup[edge.source]
            target_idx, target_layer = self._lookup[edge.target]

            # swap indices to enforce ordering
            if source_layer > target_layer:
                source_layer, target_layer = target_layer, source_layer
                source_idx, target_idx = target_idx, source_idx

            if source_layer not in self._interlayer_edges:
                self._interlayer_edges[source_layer] = {target_layer: []}

            if target_layer not in self._interlayer_edges[source_layer]:
                self._interlayer_edges[source_layer][target_layer] = []

            self._interlayer_edges[source_layer][target_layer].append(
                [source_idx, target_idx]
            )

        self._interlayer_edges = {
            s: {t: np.array(edges, dtype=np.int64) for t, edges in c.items()}
            for s, c in self._interlayer_edges.items()
        }

    def pos(self, layer_key, height=None):
        if layer_key not in self._pos:
            return None

        pos = self._pos[layer_key].copy()
        if height:
            pos[:, 2] += height

        return pos

    @property
    def edges(self):
        return self._interlayer_edges

    def layer_edges(self, layer_key):
        return self._edges.get(layer_key)


@dataclass
class LayerConfig:
    colormode: ColorMode = ColorMode.LAYER
    height_scale: float = 5.0
    draw_nodes: bool = True
    draw_edges: bool = True


class LayerHandle:
    def __init__(self, config, server, key):
        """Add options for layer to viser."""
        self.key = key
        self.name = _layer_name(key)
        self._colormode = config.colormode

        self._nodes = None
        self._edges = None

        with server.gui.add_folder(self.name):
            self._height = server.gui.add_number(
                "height", initial_value=config.height_scale * key.layer
            )
            self._draw_nodes = server.gui.add_checkbox(
                "draw_nodes", initial_value=config.draw_nodes
            )
            self._draw_edges = server.gui.add_checkbox(
                "draw_edges", initial_value=config.draw_edges
            )

    @property
    def height(self):
        return self._height.value

    @property
    def draw_nodes(self):
        return self._draw_nodes.value

    @property
    def color_mode(self):
        return self._colormode

    def update(self):
        draw_edges = self._draw_nodes.value and self._draw_edges.value
        if self._nodes:
            self._nodes.visible = self._draw_nodes.value

        if self._edges:
            self._edges.visible = draw_edges

    def draw(self, server, G, layer, view, colormap):
        pos = view.pos(layer.key, self.height)
        if pos is None:
            return

        colors = np.zeros(pos.shape)
        for idx, node in enumerate(layer.nodes):
            colors[idx] = colormap(G, node).to_float_array()

        self._nodes = server.scene.add_point_cloud(
            f"{self.name}_nodes", pos, colors=colors, point_size=0.1
        )

        edge_indices = view.layer_edges(layer.key)
        if edge_indices is not None:
            self._edges = server.scene.add_line_segments(
                f"{self.name}_edges", pos[edge_indices], (0.0, 0.0, 0.0)
            )


class ViserRenderer:
    def __init__(self, ip="localhost"):
        self._server = viser.ViserServer(host=ip)
        self._handles = {}
        self._edge_handles = {}

    def draw_mesh(self, mesh):
        vertices = mesh.get_vertices()
        mesh = trimesh.Trimesh(
            vertices=vertices[:3, :].T,
            faces=mesh.get_faces().T,
            visual=trimesh.visual.ColorVisuals(vertex_colors=vertices[3:, :].T),
        )
        self._server.scene.add_mesh_trimesh(name="/mesh", mesh=mesh)

    def draw(self, G):
        if G.has_mesh():
            self.draw_mesh(G.mesh)

        view = FlatGraphView(G)
        color_modes = {}
        for layer in itertools.chain(G.layers, G.layer_partitions):
            if layer.key not in self._handles:
                self._handles[layer.key] = LayerHandle(
                    LayerConfig(), self._server, layer.key
                )
            color_modes[layer.key] = self._handles[layer.key].color_mode

        colormaps = colormap_from_modes(color_modes)
        for layer in itertools.chain(G.layers, G.layer_partitions):
            self._handles[layer.key].draw(
                self._server, G, layer, view, colormaps[layer.key]
            )

        self._draw_interlayer_edges(G, view)

    def update(self):
        for layer_key, handle in self._handles.items():
            handle.update()

        for source_key, targets in self._edge_handles.items():
            for target_key, handle in targets.items():
                handle.visible = (
                    self._handles[source_key].draw_nodes
                    and self._handles[target_key].draw_nodes
                )

    def _draw_interlayer_edges(self, G, view):
        for source_layer, targets in view.edges.items():
            self._edge_handles[source_layer] = {}
            for target_layer, edge_indices in targets.items():
                source_pos = view.pos(source_layer, self._handles[source_layer].height)
                target_pos = view.pos(target_layer, self._handles[target_layer].height)
                assert source_pos is not None and target_pos is not None

                source_name = _layer_name(source_layer)
                target_name = _layer_name(target_layer)
                pos = np.vstack((source_pos, target_pos))

                edge_indices = edge_indices.copy()
                edge_indices[:, 1] += source_pos.shape[0]

                self._edge_handles[source_layer][target_layer] = (
                    self._server.scene.add_line_segments(
                        f"{source_name}_to_{target_name}",
                        pos[edge_indices],
                        (0.0, 0.0, 0.0),
                    )
                )


@click.command("visualize")
@click.argument("filepath", type=click.Path(exists=True))
@click.option("--ip", default="localhost")
def cli(filepath, ip):
    """Visualize a scene graph from FILEPATH using Open3D."""
    renderer = ViserRenderer(ip)
    G = dsg.DynamicSceneGraph.load(filepath)
    renderer.draw(G)
    while True:
        time.sleep(0.1)
        renderer.update()
