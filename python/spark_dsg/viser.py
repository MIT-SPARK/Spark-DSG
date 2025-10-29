"""Entry point for visualizing scene graph."""

import enum
import functools
import itertools
from dataclasses import dataclass

import numpy as np
import trimesh
import viser

import spark_dsg as dsg


class ColorMode(enum.Enum):
    LAYER = "layer"
    ID = "id"
    LABEL = "label"
    PARENT = "parent"


def _layer_name(layer_key):
    return f"layer_{layer_key.layer}p{layer_key.partition}"


def color_from_label(G, node, default=None):
    if not isinstance(node.attributes, dsg.SemanticNodeAttributes):
        return default or dsg.Color()

    return dsg.distinct_150_color(node.attributes.semantic_label)


def color_from_id(G, node):
    return dsg.colorbrewer_color(node.id.category_id)


def color_from_parent(G, node, parent_func, default=None):
    if not node.has_parent():
        return default or dsg.Color()

    return parent_func(G, G.get_node(node.get_parent()))


def color_from_layer(G, node):
    return dsg.rainbow_color(node.layer.layer)


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
                color_from_parent,
                parent_func=lambda G, x: colormap[x.layer](G, x),
                default=default,
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
    node_scale: float = 0.2
    edge_scale: float = 0.1
    draw_nodes: bool = True
    draw_edges: bool = True
    draw_labels: bool = False


DEFAULT_CONFIG = {
    dsg.LayerKey(2): LayerConfig(node_scale=0.25, draw_labels=True),
    dsg.LayerKey(3): LayerConfig(node_scale=0.1),
    dsg.LayerKey(3, 1): LayerConfig(node_scale=0.1),
    dsg.LayerKey(3, 2): LayerConfig(node_scale=0.1),
    dsg.LayerKey(4): LayerConfig(node_scale=0.4, draw_labels=True),
    dsg.LayerKey(5): LayerConfig(draw_nodes=False),
}

DEFAULT_COLORMODES = {
    dsg.LayerKey(2): ColorMode.LABEL,
    dsg.LayerKey(3): ColorMode.PARENT,
    dsg.LayerKey(3, 1): ColorMode.LABEL,
    dsg.LayerKey(3, 2): ColorMode.LAYER,
    dsg.LayerKey(4): ColorMode.ID,
    dsg.LayerKey(5): ColorMode.ID,
}


@dataclass
class LabelInfo:
    name: str
    text: str
    pos: np.ndarray


class LayerHandle:
    """Viser handles to layer elements and gui settings."""

    def __init__(
        self, server, config, colormap, height, G, layer, view, parent_callback
    ):
        """Add options for layer to viser."""
        self.key = layer.key
        self.name = _layer_name(layer.key)
        self._parent_callback = parent_callback
        self._server = server

        self._folder = server.gui.add_folder(self.name)
        with self._folder:
            self._draw_nodes = server.gui.add_checkbox(
                "draw_nodes", initial_value=config.draw_nodes
            )
            self._draw_labels = server.gui.add_checkbox(
                "draw_labels", initial_value=config.draw_labels
            )
            self._draw_edges = server.gui.add_checkbox(
                "draw_edges", initial_value=config.draw_edges
            )
            self._node_scale = server.gui.add_number(
                "node_scale", initial_value=config.node_scale
            )
            self._edge_scale = server.gui.add_number(
                "edge_scale", initial_value=config.edge_scale
            )

        self._nodes = None
        self._edges = None
        pos = view.pos(layer.key, height)
        if pos is None:
            return

        colors = np.zeros(pos.shape)
        for idx, node in enumerate(layer.nodes):
            colors[idx] = colormap(G, node).to_float_array()

        self._nodes = server.scene.add_point_cloud(
            f"{self.name}_nodes", pos, colors=colors
        )

        self._label_info = []
        self._label_handles = []
        labelspace = G.get_labelspace(self.key.layer, self.key.partition)
        for idx, node in enumerate(layer.nodes):
            text = node.id.str(literal=False)
            if labelspace:
                text += ": " + labelspace.get_node_category(node)

            self._label_info.append(
                LabelInfo(name=f"label_{node.id.str()}", text=text, pos=pos[idx])
            )

        edge_indices = view.layer_edges(layer.key)
        self._edges = None
        if edge_indices is not None:
            self._edges = server.scene.add_line_segments(
                f"{self.name}_edges",
                pos[edge_indices],
                (0.0, 0.0, 0.0),
            )

        self._update()
        self._draw_nodes.on_update(lambda _: self._update())
        self._draw_labels.on_update(lambda _: self._update())
        self._draw_edges.on_update(lambda _: self._update())
        self._node_scale.on_update(lambda _: self._update())
        self._edge_scale.on_update(lambda _: self._update())

    @property
    def draw_nodes(self):
        return self._draw_nodes.value

    @property
    def color_mode(self):
        return self._colormode

    def _update(self):
        draw_edges = self._draw_nodes.value and self._draw_edges.value
        draw_labels = self._draw_nodes.value and self._draw_labels.value
        labels_drawn = len(self._label_handles) > 0

        self._nodes.visible = self._draw_nodes.value
        self._nodes.point_size = self._node_scale.value
        if self._edges:
            self._edges.visible = draw_edges
            self._edges.line_width = self._edge_scale.value

        if not draw_labels and labels_drawn:
            for x in self._label_handles:
                x.remove()

            self._label_handles = []

        if draw_labels and not labels_drawn:
            self._label_handles = [
                self._server.scene.add_label(x.name, x.text, position=x.pos)
                for x in self._label_info
            ]

        self._parent_callback()

    def remove(self):
        if self._nodes:
            self._nodes.remove()
        if self._edges:
            self._edges.remove()

        for x in self._label_handles:
            x.remove()

        self._label_handles = []

        self._draw_nodes.remove()
        self._draw_edges.remove()
        self._node_scale.remove()
        self._edge_scale.remove()
        self._folder.remove()


class GraphHandle:
    """Visualization handles for a scene graph."""

    def __init__(self, server, G, height_scale=5.0):
        """Draw a scene graph in the visualizer."""
        self._handles = {}
        self._edge_handles = {}
        self._height_scale = height_scale
        self._edge_scale = server.gui.add_number(
            "interlayer_edge_scale", initial_value=0.1
        )

        color_modes = {}
        for layer in itertools.chain(G.layers, G.layer_partitions):
            color_modes[layer.key] = DEFAULT_COLORMODES.get(layer.key, ColorMode.LAYER)

        colormaps = colormap_from_modes(color_modes)

        view = FlatGraphView(G)
        for layer in itertools.chain(G.layers, G.layer_partitions):
            self._handles[layer.key] = LayerHandle(
                server,
                DEFAULT_CONFIG.get(layer.key, LayerConfig()),
                colormaps[layer.key],
                self._layer_height(layer.key),
                G,
                layer,
                view,
                self._update,
            )

        for source_layer, targets in view.edges.items():
            self._edge_handles[source_layer] = {}
            for target_layer, edge_indices in targets.items():
                source_pos = view.pos(source_layer, self._layer_height(source_layer))
                target_pos = view.pos(target_layer, self._layer_height(target_layer))
                assert source_pos is not None and target_pos is not None

                source_name = _layer_name(source_layer)
                target_name = _layer_name(target_layer)
                pos = np.vstack((source_pos, target_pos))

                edge_indices = edge_indices.copy()
                edge_indices[:, 1] += source_pos.shape[0]

                self._edge_handles[source_layer][target_layer] = (
                    server.scene.add_line_segments(
                        f"{source_name}_to_{target_name}",
                        pos[edge_indices],
                        (0.0, 0.0, 0.0),
                    )
                )

        self._update()
        self._edge_scale.on_update(lambda _: self._update())

    def remove(self):
        """Remove graph elements from the visualizer."""
        self._edge_scale.remove()
        for _, handle in self._handles.items():
            handle.remove()

        self._handles = {}

        for _, handles in self._edge_handles.items():
            for _, handle in handles.items():
                handle.remove()

        self._edge_handles = {}

    def _layer_height(self, layer_key):
        return self._height_scale * layer_key.layer

    def _update(self):
        for source_key, targets in self._edge_handles.items():
            for target_key, handle in targets.items():
                handle.visible = (
                    self._handles[source_key].draw_nodes
                    and self._handles[target_key].draw_nodes
                )
                handle.line_width = self._edge_scale.value


class MeshHandle:
    """Visualizer handle for mesh elements."""

    def __init__(self, server, mesh):
        """Send a mesh to the visualizer."""
        vertices = mesh.get_vertices()
        mesh = trimesh.Trimesh(
            vertices=vertices[:3, :].T,
            faces=mesh.get_faces().T,
            visual=trimesh.visual.ColorVisuals(vertex_colors=vertices[3:, :].T),
        )

        self._mesh_handle = server.scene.add_mesh_trimesh(name="/mesh", mesh=mesh)

    def remove(self):
        """Remove mesh elements from the visualizer."""
        self._mesh_handle.remove()


class ViserRenderer:
    """Rendering interface to Viser client."""

    def __init__(self, ip="localhost", port=8080, clear_at_exit=True):
        self._server = viser.ViserServer(host=ip, port=port)
        self._clear_at_exit = clear_at_exit
        self._mesh_handle = None
        self._graph_handle = None

    def __enter__(self):
        """Enter a context manager."""
        return self

    def __exit__(self, typ, exc, tb):
        """Clean up visualizer on exit if desired."""
        print(f"typ='{typ}' exc='{exc}' tb='{tb}'")
        if self._clear_at_exit:
            self.clear()

        return True

    def draw(self, G, height_scale=5.0):
        self._clear_graph()
        self._graph_handle = GraphHandle(self._server, G, height_scale=height_scale)

        if G.has_mesh():
            self.draw_mesh(G.mesh)

    def draw_mesh(self, mesh):
        self._clear_mesh()
        self._mesh_handle = MeshHandle(self._server, mesh)

    def clear(self):
        """Remove all graph and mesh elements from the visualizer."""
        self._clear_mesh()
        self._clear_graph()

    def _clear_mesh(self):
        if self._mesh_handle:
            self._mesh_handle.remove()
            self._mesh_handle = None

    def _clear_graph(self):
        if self._graph_handle:
            self._graph_handle.remove()
            self._graph_handle = None
