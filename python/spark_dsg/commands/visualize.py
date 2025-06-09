"""Entry point for visualizing scene graph."""

import itertools
import time

import click
import numpy as np
import spark_dsg as dsg
import trimesh
import viser


def _layer_name(layer_key):
    return f"layer_{layer_key.layer}p{layer_key.partition}"


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

    def pos(self, layer_key):
        return self._pos.get(layer_key)

    @property
    def edges(self):
        return self._interlayer_edges

    def layer_edges(self, layer_key):
        return self._edges.get(layer_key)


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
        if G.has_mesh():
            self.draw_mesh(G.mesh)

        view = FlatGraphView(G)
        pos_buffer = {}
        for layer in itertools.chain(G.layers, G.layer_partitions):
            pos_buffer[layer.key] = self._draw_layer(G, layer, view)

        self._draw_interlayer_edges(G, view, pos_buffer)

    def _draw_layer(self, G, layer, view):
        name = _layer_name(layer.key)
        if layer.num_nodes() == 0:
            return

        pos = view.pos(layer.key).copy()
        pos[:, 2] += 3.0 * layer.id
        self._server.scene.add_point_cloud(
            f"{name}_nodes", pos, colors=(0.0, 0.0, 0.0), point_size=0.1
        )

        edge_indices = view.layer_edges(layer.key)
        if edge_indices is not None:
            self._server.scene.add_line_segments(
                f"{name}_edges", pos[edge_indices], (0.0, 0.0, 0.0)
            )

        return pos

    def _draw_interlayer_edges(self, G, view, pos_buffer):
        for source_layer, targets in view.edges.items():
            for target_layer, edge_indices in targets.items():
                if source_layer not in pos_buffer or target_layer not in pos_buffer:
                    continue

                source_name = _layer_name(source_layer)
                target_name = _layer_name(target_layer)
                source_pos = pos_buffer[source_layer]
                pos = np.vstack((source_pos, pos_buffer[target_layer]))

                edge_indices = edge_indices.copy()
                edge_indices[:, 1] += source_pos.shape[0]
                self._server.scene.add_line_segments(
                    f"interlayer_edges_{source_name}_{target_name}",
                    pos[edge_indices],
                    (0.0, 0.0, 0.0),
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
