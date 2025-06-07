"""Entry point for visualizing scene graph."""

import time

import click
import numpy as np
import spark_dsg as dsg
import trimesh
import viser


class ViserRenderer:
    def __init__(self, ip="localhost"):
        self._server = viser.ViserServer(host=ip)

    def draw_mesh(self, mesh):
        vertices = mesh.get_vertices()
        mesh = trimesh.Trimesh(
            vertices=vertices[:3, :].T,
            faces=mesh.get_faces().T,
            visual=trimesh.visual.ColorVisuals(vertex_colors=vertices[3:, :].T),
        )
        self._server.scene.add_mesh_trimesh(name="/mesh", mesh=mesh)

    def draw(self, G):
        self._nodes = {}
        self._edges = {}
        self._interlayer_edges = {}

        if G.has_mesh():
            self.draw_mesh(G.mesh)

        self._init_positions(G)
        for layer in G.layers:
            self._draw_layer(layer)

        for layer in G.layer_partitions:
            self._draw_layer(layer)

        self._draw_interlayer_edges(G)

    def _init_positions(self, G):
        self._pos = np.zeros((G.num_nodes(), 3))
        self._lookup = {}
        self._layer_start = {}

        offset = 0
        for layer in G.layers:
            self._layer_start[layer.key] = offset
            for idx, node in enumerate(layer.nodes):
                self._pos[idx + offset, :] = node.attributes.position
                self._lookup[node.id.value] = idx + offset

            self._pos[offset : offset + layer.num_nodes(), 2] += 3.0 * layer.id
            offset += layer.num_nodes()

        for partition in G.layer_partitions:
            self._layer_start[partition.key] = offset
            for idx, node in enumerate(partition.nodes):
                self._pos[idx + offset, :] = node.attributes.position
                self._lookup[node.id.value] = idx + offset

            self._pos[offset : offset + partition.num_nodes(), 2] += 3.0 * partition.id
            offset += partition.num_nodes()

    def _draw_layer(self, layer):
        if layer.num_nodes() == 0:
            return

        start_idx = self._layer_start[layer.key]
        pos = self._pos[start_idx : start_idx + layer.num_nodes()]

        name = f"layer_{layer.id}p{layer.partition}"
        self._server.scene.add_point_cloud(
            f"{name}_nodes", pos, colors=(0.0, 0.0, 0.0), point_size=0.1
        )

        if layer.num_edges() == 0:
            return

        edge_points = np.zeros((layer.num_edges(), 2, 3))
        for idx, edge in enumerate(layer.edges):
            edge_points[idx, 0, :] = self._pos[self._lookup[edge.source], :]
            edge_points[idx, 1, :] = self._pos[self._lookup[edge.target], :]

        self._server.scene.add_line_segments(
            f"{name}_edges", edge_points, (0.0, 0.0, 0.0)
        )

    def _draw_interlayer_edges(self, G):
        edge_points = []
        for idx, edge in enumerate(G.interlayer_edges):
            p_edge = [
                self._pos[self._lookup[edge.source], :],
                self._pos[self._lookup[edge.target], :],
            ]
            edge_points.append(p_edge)

        if len(edge_points) == 0:
            return

        edge_points = np.array(edge_points)
        self._server.scene.add_line_segments(
            "interlayer_edges", edge_points, (0.0, 0.0, 0.0)
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
