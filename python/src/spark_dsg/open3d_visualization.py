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
"""Scene Graph visualizer using open3d."""
from spark_dsg._dsg_bindings import DsgLayers, NodeSymbol, DynamicSceneGraph
import multiprocessing as mp
import numpy as np
import time

try:
    import open3d as o3d
    import seaborn as sns
    from scipy.spatial.transform import Rotation as Rot
    import zmq

    OPEN3D_VISUALIZER_ENABLED = True
except ImportError as e:
    OPEN3D_VISUALIZER_ENABLED = False
    import logging

    logging.warning(f"could not import open3d visualizer dependencies: {e}")


LAYER_OFFSETS = {
    DsgLayers.OBJECTS: 0.0,
    DsgLayers.PLACES: 6.0,
    DsgLayers.ROOMS: 12.0,
    DsgLayers.BUILDINGS: 16.0,
}

LAYER_IDS = (DsgLayers.OBJECTS, DsgLayers.PLACES, DsgLayers.ROOMS, DsgLayers.BUILDINGS)
CATEGORY_MAP = {
    "O": DsgLayers.OBJECTS,
    "p": DsgLayers.PLACES,
    "R": DsgLayers.ROOMS,
    "B": DsgLayers.BUILDINGS,
}

LAYER_NAMES = {
    DsgLayers.OBJECTS: "objects",
    DsgLayers.PLACES: "places",
    DsgLayers.ROOMS: "rooms",
    DsgLayers.BUILDINGS: "buildings",
}


def _get_edge_layers(edge):
    source_id = NodeSymbol(edge.source)
    target_id = NodeSymbol(edge.target)
    return CATEGORY_MAP.get(source_id.category), CATEGORY_MAP.get(target_id.category)


class RemoteVisualizer:
    """Python visualizer."""

    def __init__(
        self,
        url="tcp://127.0.0.1:8001",
        point_size=7.5,
        num_place_edges_to_skip=5,
        background_color=(0.4, 0.4, 0.4),
        use_mesh=True,
        use_mesh_normals=False,
        include_dynamic=True,
        num_dynamic_to_skip=3,
        dynamic_axes_size=0.2,
        layers_to_skip=[],
        collapse_layers=False,
    ):
        """Make the visualizer geometries."""
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(url)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self._point_size = point_size
        self._num_place_edges_to_skip = num_place_edges_to_skip
        self._background_color = background_color
        self._use_mesh = use_mesh
        self._use_mesh_normals = use_mesh_normals
        self._include_dynamic = include_dynamic
        self._num_dynamic_to_skip = num_dynamic_to_skip
        self._dynamic_axes_size = dynamic_axes_size
        self._layers_to_skip = layers_to_skip
        self._collapse_layers = collapse_layers

        self._palette = sns.color_palette("Paired")
        self._num_colors = len(self._palette)

        self._geometries = {}
        self._new_geometries = []

    def _get_geometry(self, name, geom_type):
        if name not in self._geometries:
            self._geometries[name] = geom_type()
            self._new_geometries.append(name)

        return self._geometries[name]

    def _update_geometries(self, G):
        if self._use_mesh:
            self._update_mesh_geometry(G)

        N = G.num_nodes(False) if self._include_dynamic else G.num_static_nodes()

        self._points = np.zeros((N, 3))
        self._colors = np.zeros((N, 3))
        self._id_map = {}

        offset = 0
        for layer in G.layers:
            if layer.id not in LAYER_IDS:
                continue

            if self._layers_to_skip and layer.id in self._layers_to_skip:
                continue

            self._update_layer_geometries(layer, offset)
            if layer.id == DsgLayers.OBJECTS:
                self._update_bounding_boxes(layer)

            offset += layer.num_nodes()

        # prune points and colors to just the nodes we've added
        self._points = self._points[:offset, :]
        self._colors = self._colors[:offset, :]
        self._update_interlayer_edges(G)

        if self._include_dynamic:
            if G.has_layer(DsgLayers.AGENTS, "a"):
                self._update_dynamic_layer(G.get_dynamic_layer(DsgLayers.AGENTS, "a"))

    def _update_mesh_geometry(self, G):
        if not G.has_mesh():
            return

        vertices = G.mesh.get_vertices()
        faces = G.mesh.get_faces()
        if vertices.size == 0 or faces.size == 0:
            return

        mesh = self._get_geometry("mesh", o3d.geometry.TriangleMesh)
        mesh.vertices = o3d.utility.Vector3dVector(vertices[:3, :].T)
        mesh.triangles = o3d.utility.Vector3iVector(faces.T)
        mesh.vertex_colors = o3d.utility.Vector3dVector(vertices[3:, :].T)

        if self._use_mesh_normals:
            mesh.compute_vertex_normals()

    def _update_bounding_boxes(self, layer):
        bboxes = self._get_geometry(
            f"bbox_{LAYER_NAMES[layer.id]}", o3d.geometry.LineSet
        )
        bboxes.clear()

        for node in layer.nodes:
            bbox = node.attributes.bounding_box
            o3d_bbox = o3d.geometry.OrientedBoundingBox(
                bbox.world_P_center.astype(np.float64),
                bbox.world_R_center.astype(np.float64),
                bbox.dimensions.astype(np.float64),
            )
            o3d_bbox.color = node.attributes.color / 255.0
            bboxes += o3d.geometry.LineSet.create_from_oriented_bounding_box(o3d_bbox)

    def _update_interlayer_edges(self, G):
        edges = []
        colors = []
        num_place_edges = 0
        for index, edge in enumerate(G.interlayer_edges):
            if edge.source not in self._id_map or edge.target not in self._id_map:
                continue

            layers = _get_edge_layers(edge)
            should_skip = DsgLayers.PLACES in layers and DsgLayers.ROOMS in layers
            if should_skip and num_place_edges % self._num_place_edges_to_skip != 0:
                num_place_edges += 1
                continue

            edges.append([self._id_map[edge.source], self._id_map[edge.target]])
            colors.append(self._get_edge_color(G, edge))

            if should_skip:
                num_place_edges += 1

        if len(edges) == 0:
            return

        line_set = self._get_geometry("interlayer_edges", o3d.geometry.LineSet)
        line_set.points = o3d.utility.Vector3dVector(self._points)
        line_set.lines = o3d.utility.Vector2iVector(edges)
        line_set.colors = o3d.utility.Vector3dVector(colors)

    def _update_layer_geometries(self, layer, offset):
        if layer.num_nodes() == 0:
            return

        for layer_idx, node in enumerate(layer.nodes):
            index = layer_idx + offset
            self._id_map[node.id.value] = index
            self._points[index, :] = node.attributes.position
            if not self._collapse_layers:
                self._points[index, 2] += LAYER_OFFSETS[layer.id]
            self._colors[index, :] = self._get_node_color(node)

        start_idx = offset
        end_idx = offset + layer.num_nodes()
        pcd = self._get_geometry(
            f"nodes_{LAYER_NAMES[layer.id]}", o3d.geometry.PointCloud
        )
        pcd.points = o3d.utility.Vector3dVector(self._points[start_idx:end_idx, :])
        pcd.colors = o3d.utility.Vector3dVector(self._colors[start_idx:end_idx, :])

        if layer.num_edges() == 0:
            return

        edges = np.zeros((layer.num_edges(), 2), dtype=np.int32)
        for index, edge in enumerate(layer.edges):
            edges[index, 0] = self._id_map[edge.source] - offset
            edges[index, 1] = self._id_map[edge.target] - offset

        edge_set = self._get_geometry(
            f"edges_{LAYER_NAMES[layer.id]}", o3d.geometry.LineSet
        )
        edge_set.points = pcd.points
        edge_set.lines = o3d.utility.Vector2iVector(edges)

    def _update_dynamic_layer(self, layer):
        if layer.num_nodes() == 0:
            return

        trajectory = self._get_geometry("dynamic_nodes", o3d.geometry.TriangleMesh)
        trajectory.clear()

        for index, node in enumerate(layer.nodes):
            if index % self._num_dynamic_to_skip != 0:
                continue

            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
                self._dynamic_axes_size
            )

            q = node.attributes.world_R_body
            world_T_body = np.eye(4)
            world_T_body[:3, :3] = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            world_T_body[:3, 3] = node.attributes.position
            axes.transform(world_T_body)
            trajectory += axes

    def _get_palette_color(self, node_id):
        return self._palette[node_id.category_id % self._num_colors]

    def _get_node_color(self, node):
        if node.layer == DsgLayers.PLACES:
            if not node.has_parent():
                return np.zeros(3)

            return self._get_palette_color(NodeSymbol(node.get_parent()))
        elif node.layer == DsgLayers.ROOMS:
            return self._get_palette_color(node.id)
        else:
            return node.attributes.color.astype(np.float64) / 255.0

    def _get_edge_color(self, G, edge):
        layers = _get_edge_layers(edge)
        if DsgLayers.PLACES in layers and DsgLayers.ROOMS in layers:
            if layers[0] == DsgLayers.ROOMS:
                return self._get_palette_color(NodeSymbol(edge.source))
            if layers[1] == DsgLayers.ROOMS:
                return self._get_palette_color(NodeSymbol(edge.target))

        if DsgLayers.PLACES in layers and DsgLayers.OBJECTS in layers:
            if layers[0] == DsgLayers.OBJECTS:
                return G.get_node(edge.source).attributes.color / 255.0
            if layers[1] == DsgLayers.OBJECTS:
                return G.get_node(edge.target).attributes.color / 255.0

        return np.zeros(3)

    def _update_render_geometry(self):
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = self._point_size
        for name, geom in self._geometries.items():
            if name in self._new_geometries:
                self._viz.add_geometry(name, geom, mat)
            else:
                self._viz.update_geometry(name, geom, mat)

        self._new_geometries = []

    def _update_names(self, G):
        self._names = [None] * len(self._id_map)
        for node_id in self._id_map:
            node = G.get_node(node_id)
            if node.layer == DsgLayers.OBJECTS or node.layer == DsgLayers.ROOMS:
                self._names[self._id_map[node_id]] = node.attributes.name

        for name, point in zip(self._names, self._points):
            if name is not None:
                self._viz.add_3d_label(point, name)

    def run(self):
        """Open a visualizer window and spin."""
        self._app = o3d.visualization.gui.Application.instance
        self._app.initialize()

        self._viz = o3d.visualization.O3DVisualizer("Scene Graph Visualizer")
        self._viz.show_settings = True
        self._viz.scene.show_skybox(False)
        self._viz.scene.set_background(list(self._background_color + (1.0,)))

        self._app.add_window(self._viz)

        while self._app.run_one_tick():
            new_msg = None
            try:
                new_msg = self._socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass

            if new_msg is None:
                continue

            if new_msg == b"shutdown":
                break

            G = DynamicSceneGraph.from_binary(new_msg)
            self._update_geometries(G)
            self._update_names(G)
            self._update_render_geometry()
            self._viz.reset_camera_to_default()

        self._viz.close()


def _run_remote_visualizer(**kwargs):
    visualizer = RemoteVisualizer(**kwargs)
    visualizer.run()


class DsgVisualizer:
    """Python visualizer."""

    def __init__(
        self,
        start_remote=True,
        url="tcp://127.0.0.1:8001",
        use_mesh=True,
        num_calls_per_update=10,
        **kwargs,
    ):
        """Make the visualizer geometries."""
        self._use_mesh = use_mesh
        self._start_remote = start_remote
        self._num_calls_per_update = num_calls_per_update
        self._num_update_calls = 0

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.bind(url)

        kwargs["url"] = url
        kwargs["use_mesh"] = use_mesh

        if self._start_remote:
            self._proc = mp.Process(target=_run_remote_visualizer, kwargs=kwargs)
            self._proc.start()
        else:
            self._proc = None

    def update_graph(self, G, force=False):
        """Set graph for remote visualizer."""
        should_update = self._num_update_calls % self._num_calls_per_update == 0
        if should_update or force:
            self._socket.send(G.to_binary(include_mesh=self._use_mesh))

        self._num_update_calls += 1

    def stop(self):
        """Send signal to remote visualizer server."""
        self._socket.send_string("shutdown")
        if self._proc is None:
            return

        self._proc.join(timeout=10)
        if self._proc.exitcode is None:
            print("[ERROR]: visualizer join timed out")
            self._proc.terminate()

    def wait(self):
        """Wait until visualizer is done."""
        if self._proc is None:
            return

        self._proc.join()


def render_to_open3d(G, block=True, url="tcp://127.0.0.1:8001", **kwargs):
    """Render graph to opend3d."""
    if not OPEN3D_VISUALIZER_ENABLED:
        logging.error("open3d is not enabled!")
        return

    viz = DsgVisualizer(url=url, **kwargs)
    time.sleep(0.5)
    viz.update_graph(G, force=True)
    if block:
        viz.wait()
