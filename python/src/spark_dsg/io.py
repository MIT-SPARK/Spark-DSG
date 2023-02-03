"""Module for reading and writing collections of DSGS."""
from spark_dsg._dsg_bindings import SceneGraphLayer
from dataclasses import dataclass
from typing import Union, Dict
import functools
import pathlib
import mmap
import json
import os


@dataclass
class IndexInfo:
    """Struct storing index information of a layer in a file."""

    index: int
    offset: int
    size: int
    num_nodes: int
    num_edges: int

    def to_json(self):
        """
        Serialize the index info to a json-compatible format.

        Returns:
            Dict[str, int]: Dictionary of fields.
        """
        return {
            "index": self.index,
            "offset": self.offset,
            "size": self.size,
            "num_nodes": self.num_nodes,
            "num_edges": self.num_edges,
        }

    @classmethod
    def from_json(cls, record):
        """
        Deserialize the index from a dictionary.

        Args:
            record (Dict[str, int]): Dictionary of fields
        """
        return cls(**record)


def requires_open(f):
    """Validate that a collection is open."""

    @functools.wraps(f)
    def wrap(self, *args, **kwargs):
        if not self.valid():
            raise RuntimeError(f"{type(self)} is not open")
        return f(self, *args, **kwargs)

    return wrap


class LayerCollection:
    """Interface for reading saved scene graph layers."""

    def __init__(self, collection_path: Union[str, pathlib.Path]):
        """
        Create a scene graph collection.

        Assumes the presence of two files: {collection_path}.graphs and
        {collection_path}.json.

        Args:
            collection_path: root path of collection
        """
        self._path = pathlib.Path(collection_path)
        self._index = None
        self._mmap = None

    def __enter__(self):
        """Open collection."""
        self.open()
        return self

    def __exit__(self, *exc):
        """Close collection."""
        self.close()

    def valid(self):
        """Return whether or not the collection has been initialized."""
        return self._index is not None and self._mmap is not None

    def open(self):
        """Open mmap and parse record."""
        record_path = self._path.with_suffix(".json")
        if not record_path.exists():
            raise FileNotFoundError(f"missing record at {record_path}")

        graph_path = self._path.with_suffix(".graphs")
        if not graph_path.exists():
            raise FileNotFoundError(f"missing graphs at {graph_path}")

        with record_path.open("r") as fin:
            record = json.load(fin)

        self._index = {}
        for index_record in record["offsets"]:
            info = IndexInfo.from_json(index_record)
            self._index[info.index] = info

        fd = os.open(str(graph_path), os.O_RDONLY)
        self._mmap = mmap.mmap(fd, 0, flags=mmap.MAP_PRIVATE, prot=mmap.PROT_READ)
        os.close(fd)

    def close(self):
        """Close open mmap."""
        if self._mmap:
            self._mmap.close()
            self._mmap = None

    def _get_graph(self, index):
        info = self._index.get(index, None)
        if not info:
            raise IndexError(f"no graph with index {index} in collection")

        start_byte = info.offset
        end_byte = info.offset + info.size
        return SceneGraphLayer.from_bson(self._mmap[start_byte:end_byte])

    @requires_open
    def __getitem__(self, index):
        """
        Get graphs from the collection.

        Args:
            index (Union[int, slice]): Graph indices to retrieve

        Returns:
            Union[SceneGraphLayer, List[SceneGraphLayer]]: Requested graph
        """
        if isinstance(index, slice):
            indices = [x for x in range(len(self))]
            indices = indices[index]
            return [self._get_graph(x) for x in indices]

        if index < 0:
            index = len(self) + index

        return self._get_graph(index)

    @property
    @requires_open
    def indices(self):
        """Get graph indices."""
        return [x for x in self._index]

    @requires_open
    def __len__(self):
        """Get number of graphs in collection."""
        return len(self._index)

    @staticmethod
    def save(collection: Dict[int, SceneGraphLayer], path: Union[str, pathlib.Path]):
        """Save a collection of graphs to disk."""
        record_path = path.with_suffix(".json")
        graph_path = path.with_suffix(".graphs")

        offsets = []
        offset = 0
        with graph_path.open("wb") as fout:
            for index, graph in collection.items():
                contents = graph.to_bson()
                fout.write(contents)
                info = IndexInfo(
                    index=index,
                    offset=offset,
                    size=len(contents),
                    num_nodes=graph.num_nodes(),
                    num_edges=graph.num_edges(),
                )

                offsets.append(info.to_json())
                offset += len(contents)

        with record_path.open("w") as fout:
            json.dump({"offsets": offsets}, fout)
