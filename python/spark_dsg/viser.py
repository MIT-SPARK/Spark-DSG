"""Entry point for visualizing scene graph."""

import enum
import functools
import itertools
import warnings
from dataclasses import dataclass

import numpy as np

import spark_dsg as dsg

BOUNDING_BOX_EDGE_INDICES = np.array(
    [[0, 0, 0, 1, 1, 3, 3, 2, 4, 4, 5, 7], [1, 3, 4, 2, 5, 2, 7, 6, 5, 7, 6, 6]],
    dtype=np.int64,
).T


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
    """General configuration for a layer in viser."""

    node_scale: float = 0.2
    edge_scale: float = 0.1
    box_width: float = 5.0
    draw_nodes: bool = True
    draw_bboxes: bool = False
    draw_labels: bool = False
    draw_edges: bool = True
    draw_interlayer: bool = True

    def init(self, server):
        """Set up config with viser server."""
        self._node_scale = server.add_number("Node Scale", self.node_scale)
        self._edge_scale = server.add_number("Edge Scale", self.edge_scale)
        self._box_width = server.add_number("Bounding Box Width", self.box_width)
        self._draw_nodes = server.add_checkbox("Nodes", self.draw_nodes)
        self._draw_boxes = server.add_checkbox("Bounding Boxes", self.draw_bboxes)
        self._draw_labels = server.add_checkbox("Labels", self.draw_labels)
        self._draw_edges = server.add_checkbox("Edges", self.draw_edges)
        self._draw_interlayer = server.add_checkbox(
            "Interlayer Edges", self.draw_interlayer
        )

    def set_callback(self, callback):
        self._node_scale.on_update(lambda _: callback())
        self._edge_scale.on_update(lambda _: callback())
        self._box_width.on_update(lambda _: callback())
        self._draw_nodes.on_update(lambda _: callback())
        self._draw_boxes.on_update(lambda _: callback())
        self._draw_labels.on_update(lambda _: callback())
        self._draw_edges.on_update(lambda _: callback())
        self._draw_interlayer.on_update(lambda _: callback())

    def remove(self):
        self._draw_nodes.remove()
        self._draw_boxes.remove()
        self._draw_labels.remove()
        self._draw_edges.remove()
        self._node_scale.remove()
        self._edge_scale.remove()
        self._draw_interlayer.remove()

    @property
    def should_draw_nodes(self):
        return self._draw_nodes.value

    @property
    def should_draw_edges(self):
        return self._draw_nodes.value and self._draw_edges.value

    @property
    def should_draw_interlayer(self):
        return self._draw_nodes.value and self._draw_interlayer.value

    @property
    def should_draw_labels(self):
        return self._draw_nodes.value and self._draw_labels.value

    @property
    def should_draw_boxes(self):
        return self._draw_nodes.value and self._draw_boxes.value

    @property
    def current_node_scale(self):
        return self._node_scale.value

    @property
    def current_edge_scale(self):
        return self._edge_scale.value

    @property
    def current_box_width(self):
        return self._box_width.value


DEFAULT_CONFIG = {
    dsg.LayerKey(2): LayerConfig(node_scale=0.25, draw_labels=True, draw_bboxes=True),
    dsg.LayerKey(3): LayerConfig(node_scale=0.1),
    dsg.LayerKey(3, 1): LayerConfig(node_scale=0.1),
    dsg.LayerKey(3, 2): LayerConfig(node_scale=0.1),
    dsg.LayerKey(4): LayerConfig(node_scale=0.4, draw_labels=True),
    dsg.LayerKey(5): LayerConfig(draw_nodes=False),
    dsg.LayerKey(2, 97): LayerConfig(node_scale=0.1, draw_interlayer=False),
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


def _layer_to_labels(G, layer, pos):
    label_info = []
    labelspace = G.get_labelspace(layer.key.layer, layer.key.partition)
    for idx, node in enumerate(layer.nodes):
        text = node.id.str(literal=False)
        if labelspace:
            text += ": " + labelspace.get_node_category(node)

        name = f"label_{node.id.str()}"
        label_info.append(LabelInfo(name=name, text=text, pos=pos[idx]))

    return label_info


def _layer_to_colors(G, layer, colormap):
    colors = np.zeros((layer.num_nodes(), 3))
    for idx, node in enumerate(layer.nodes):
        colors[idx] = colormap(G, node).to_float_array()

    return colors


def _layer_to_boxes(layer, pos, colors):
    N_EDGES = 13
    num_valid = 0
    bb_pos = np.zeros((N_EDGES * layer.num_nodes(), 2, 3))
    bb_color = np.zeros((N_EDGES * layer.num_nodes(), 2, 3))
    for idx, node in enumerate(layer.nodes):
        attrs = node.attributes
        if not isinstance(attrs, dsg.SemanticNodeAttributes):
            continue

        if not attrs.bounding_box.is_valid():
            continue

        start_idx = N_EDGES * num_valid
        end_idx = start_idx + N_EDGES
        corners = np.array(attrs.bounding_box.corners())
        bb_pos[start_idx : end_idx - 1] = corners[BOUNDING_BOX_EDGE_INDICES]
        bb_pos[end_idx - 1, 0, :] = pos[idx]
        bb_pos[end_idx - 1, 1, :] = attrs.bounding_box.world_P_center
        bb_color[start_idx:end_idx, :] = colors[idx, :]
        num_valid += 1

    final_idx = N_EDGES * num_valid
    return bb_pos[:final_idx], bb_color[:final_idx]


class LayerHandle:
    """Viser handles to layer elements and gui settings."""

    def __init__(
        self, server, config, colormap, height, G, layer, view, parent_callback
    ):
        """Add options for layer to viser."""
        self.key = layer.key
        self.name = _layer_name(layer.key)
        self.config = config

        self._parent_callback = parent_callback
        self._server = server
        self._folder = server.add_folder(self.name, expand_by_default=False)

        with self._folder:
            self.config.init(self._server)

        self._nodes = None
        self._edges = None
        self._boxes = None
        self._labels = []
        self._label_handles = []

        pos = view.pos(layer.key, height)
        edges = view.layer_edges(layer.key)
        if pos is None:
            return

        name = self.name
        colors = _layer_to_colors(G, layer, colormap)
        bb_info = _layer_to_boxes(layer, pos, colors)

        self._labels = _layer_to_labels(G, layer, pos)
        self._nodes = server.scene.add_point_cloud(f"{name}_nodes", pos, colors=colors)
        self._boxes = server.add_line_segments(f"{name}_boxes", bb_info[0], bb_info[1])

        if edges is not None:
            self._edges = server.scene.add_line_segments(
                f"{name}_edges", pos[edges], (0.0, 0.0, 0.0)
            )

        self._update()
        self.config.set_callback(self._update)

    @property
    def draw_nodes(self):
        return self.config.should_draw_nodes

    @property
    def color_mode(self):
        return self._colormode

    def _draw_labels(self):
        self._label_handles = [
            self._server.add_label(x.name, x.text, position=x.pos) for x in self._labels
        ]

    def _remove_labels(self):
        for x in self._label_handles:
            x.remove()

        self._label_handles = []

    def _update(self):
        if self._nodes:
            self._nodes.visible = self.config.should_draw_nodes
            self._nodes.point_size = self.config.current_node_scale

        if self._edges:
            self._edges.visible = self.config.should_draw_edges
            self._edges.line_width = self.config.current_edge_scale

        if self._boxes:
            self._boxes.visible = self.config.should_draw_boxes
            self._boxes.line_width = self.config.current_box_width

        labels_drawn = len(self._label_handles) > 0
        if not self.config.should_draw_labels and labels_drawn:
            self._remove_labels()

        if self.config.should_draw_labels and not labels_drawn:
            self._draw_labels()

        self._parent_callback()

    def remove(self):
        if self._nodes:
            self._nodes.remove()

        if self._edges:
            self._edges.remove()

        if self._boxes:
            self._boxes.remove()

        self.config.remove()
        self._remove_labels()
        self._folder.remove()


class GraphHandle:
    """Visualization handles for a scene graph."""

    def __init__(self, server, G, height_scale=5.0):
        """Draw a scene graph in the visualizer."""
        self._handles = {}
        self._edge_handles = {}
        self._height_scale = height_scale
        self._edge_scale = server.gui.add_number(
            "Interlayer Edge Scale", initial_value=0.3
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
                    self._handles[source_key].config.should_draw_interlayer
                    and self._handles[target_key].config.should_draw_interlayer
                )
                handle.line_width = self._edge_scale.value


class MeshHandle:
    """Visualizer handle for mesh elements."""

    def __init__(self, server, mesh):
        """Send a mesh to the visualizer."""
        try:
            import trimesh

            vertices = mesh.get_vertices()
            mesh = trimesh.Trimesh(
                vertices=vertices[:3, :].T,
                faces=mesh.get_faces().T,
                visual=trimesh.visual.ColorVisuals(vertex_colors=vertices[3:, :].T),
            )

            self._mesh_handle = server.scene.add_mesh_trimesh(
                name="/mesh", mesh=mesh, cast_shadow=False, receive_shadow=False
            )
        except ImportError:
            warnings.warn("Missing [viz] deps (trimesh)! Reinstall with spark_dsg[viz]")
            self._mesh_handle = None

    def remove(self):
        """Remove mesh elements from the visualizer."""
        if self._mesh_handle is not None:
            self._mesh_handle.remove()


class ViserRenderer:
    """Scene graph renderer using [viser](."""

    def __init__(
        self, ip: str = "localhost", port: int = 8080, clear_at_exit: bool = True
    ):
        """
        Construct a handle to a viser server used for drawing a scene graph.

        Args:
            ip: IP for the server to bind to
            port: Port for the server to bind to
            clear_at_exit: Remove scene graph elements from viser client when server exits
        """
        self._mesh_handle = None
        self._graph_handle = None
        self._clear_at_exit = clear_at_exit

        try:
            import viser

            self._server = viser.ViserServer(host=ip, port=port)
        except ImportError:
            warnings.warn("Missing [viz] deps (viser)! Reinstall with spark_dsg[viz]")
            self._server = None

    def __enter__(self):
        """Enable context manager for clearing viser client on exit from renderer scope."""
        return self

    def __exit__(self, typ, exc, tb):
        """Handle clearing client on exit from renderer scope."""
        print(f"typ='{typ}' exc='{exc}' tb='{tb}'")
        if self._clear_at_exit and self._server:
            self.clear()

        return True

    def draw(self, G: dsg.DynamicSceneGraph, height_scale: float = 2.0):
        """
        Render a scene graph to viser (requires [viz] extra).

        Args:
            G: Graph to draw
            height_scale: z-separation between layers
        """
        if self._server is None:
            warnings.warn("Visualization disabled because of missing deps!")
            return

        self._clear_graph()
        self._graph_handle = GraphHandle(self._server, G, height_scale=height_scale)
        if G.has_mesh():
            self.draw_mesh(G.mesh)

    def draw_mesh(self, mesh):
        """
        Render a mesh to viser (requires [viz] extra).

        Args:
            mesh: Mesh to draw
        """
        if self._server is None:
            warnings.warn("Visualization disabled because of missing deps!")
            return

        self._clear_mesh()
        self._mesh_handle = MeshHandle(self._server, mesh)

    def clear(self):
        """Remove all graph and mesh elements from the visualizer."""
        if self._server is None:
            warnings.warn("Visualization disabled because of missing deps!")
            return

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
