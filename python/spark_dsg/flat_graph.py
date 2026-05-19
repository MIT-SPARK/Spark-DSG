import itertools

import numpy as np

from spark_dsg._dsg_bindings import (
    SceneGraph,
    LayerKey,
    LayerView,
    SceneGraphLayer,
    NodeSymbol,
    NodeAttributes,
)


class FlatLayerView:
    """Build a tensor representation of a scene layer."""

    def __init__(self, layer: LayerView | SceneGraphLayer):
        self._lookup = {}

        ids = []
        attrs = []
        pos = np.zeros((layer.num_nodes(), 3))
        for idx, node in enumerate(layer.nodes):
            pos[idx, :] = node.attributes.position
            ids.append(node.id.value)
            attrs.append(node.attributes)
            self._lookup[node.id.value] = idx

        edge_tensor = np.zeros((layer.num_edges(), 2), dtype=np.int64)
        for idx, edge in enumerate(layer.edges):
            edge_tensor[idx, 0] = self._lookup[edge.source]
            edge_tensor[idx, 1] = self._lookup[edge.target]

        self._node_ids = np.array(ids)
        self._pos = pos
        self._attributes = attrs
        self._edges = edge_tensor

    @property
    def ids(self) -> np.ndarray:
        return self._node_ids

    @property
    def node_symbols(self) -> list[NodeSymbol]:
        return [NodeSymbol(x) for x in self._node_ids]

    @property
    def positions(self) -> np.ndarray:
        return self._pos

    @property
    def attributes(self) -> list[NodeAttributes]:
        return self._attributes

    @property
    def edges(self) -> np.ndarray:
        return self._edges


class FlatGraphView:
    """Build a tensor representation of the scene graph."""

    def __init__(self, G: SceneGraph):
        self._layers = {}
        self._lookup = {}
        for layer in itertools.chain(G.layers, G.layer_partitions):
            if layer.num_nodes() == 0:
                continue

            view = FlatLayerView(layer)
            self._layers[layer.key] = view
            for idx, node_id in enumerate(view.ids):
                self._lookup[node_id] = (layer.key, idx)

        self._interlayer_edges = {}
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

    @property
    def edges(self):
        return self._interlayer_edges

    def layer(self, layer_key: LayerKey) -> FlatLayerView | None:
        """Get edges for a particular layer."""
        return self._layers.get(layer_key)
