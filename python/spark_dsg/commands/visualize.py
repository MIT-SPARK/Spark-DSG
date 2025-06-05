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

    def draw(self, G):
        self.draw_mesh(G)
        for idx, layer in enumerate(G.layers):
            self.draw_layer(layer, 3 * idx)

    def draw_layer(self, layer, height):
        if layer.num_nodes() == 0:
            return

        pos = []
        lookup = {}
        for idx, node in enumerate(layer.nodes):
            pos.append(node.attributes.position)
            lookup[node.id] = idx

        pos = np.squeeze(np.array(pos))
        self._server.add_point_cloud(
            f"layer_{layer.id}_nodes", pos, colors=(0.0, 0.0, 0.0), point_size=0.1
        )

    def draw_mesh(self, G):
        if not G.has_mesh():
            return

        vertices = G.mesh.get_vertices()
        mesh = trimesh.Trimesh(
            vertices=vertices[:3, :].T,
            faces=G.mesh.get_faces().T,
            visual=trimesh.visual.ColorVisuals(vertex_colors=vertices[3:, :].T),
        )
        self._server.scene.add_mesh_trimesh(name="/mesh", mesh=mesh)


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
