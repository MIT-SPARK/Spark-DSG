"""Module containing python equivalent of c++ zmq interface."""

import threading
import warnings
from typing import Optional

import zmq

from spark_dsg._dsg_bindings import DynamicSceneGraph


def _get_context(context: Optional[zmq.Context], num_threads: Optional[int]):
    to_return = context or zmq.Context.instance()
    if num_threads:
        to_return.set(zmq.IO_THREADS, num_threads)

    return to_return


class DsgSender:
    """Publish scene graph over ZMQ."""

    def __init__(
        self,
        url: str,
        num_threads: Optional[int] = None,
        context: Optional[zmq.Context] = None,
    ):
        """Initialize the ZMQ socket."""
        self._context = _get_context(context, num_threads)
        self._socket = self._context.socket(zmq.PUB)
        self._socket.bind(url)

    def send(self, graph: DynamicSceneGraph, include_mesh=False):
        """Publish the scene graph over ZMQ."""
        self._socket.send(graph.to_binary(include_mesh=include_mesh))


class DsgReceiver:
    """Receive a scene graph over ZMQ."""

    def __init__(
        self,
        url: str,
        num_threads: Optional[int] = None,
        conflate: bool = True,
        context: Optional[zmq.Context] = None,
    ):
        """Initialize the ZMQ socket."""
        self._context = _get_context(context, num_threads)
        self._graph = None

        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(url)

        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        if conflate:
            self._socket.setsockopt(zmq.CONFLATE, True)

    def recv(self, timeout_ms: int, recv_all: Optional[bool] = None) -> bool:
        """Receive a scene graph."""
        if recv_all is not None:
            warnings.warn(
                "'recv_all' is deprecated. use 'conflate' when making the receiver",
                DeprecationWarning,
                stacklevel=2,
            )

        if not self._socket.poll(timeout_ms):
            return False

        buf = self._socket.recv()
        if self._graph is None:
            self._graph = DynamicSceneGraph.from_binary(buf)
        else:
            self._graph.update_from_binary(buf)

        return True

    @property
    def graph(self) -> DynamicSceneGraph:
        """Return the latest scene graph."""
        if self._graph is None:
            raise ValueError("no graph received yet")

        return self._graph


class ZmqGraph:
    """A wrapper around a scene graph received from ZMQ in the background."""

    def __init__(
        self,
        url: str,
        num_threads: Optional[int] = None,
        poll_time_ms: int = 100,
        context: Optional[zmq.Context] = None,
    ):
        """Initialize a ZMQ-backed scene graph."""
        self._receiver = DsgReceiver(
            url, num_threads=num_threads, conflate=True, context=context
        )
        self._poll_time_ms = poll_time_ms
        self._has_change = False
        self._graph = None

        self._mutex = threading.Lock()
        self._should_shutdown = threading.Event()
        self._thread = None

    def start(self):
        """Start the background thread."""
        self._thread = threading.Thread(target=self._poll_thread)
        self._thread.start()

    def stop(self):
        """Stop the background thread."""
        if self._thread is None:
            return

        self._should_shutdown.set()
        self._thread.join()
        self._thread = None

    def __enter__(self):
        """Start the background thread."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_msg, tb):
        """Stop the background thread."""
        self.stop()

    @property
    def has_change(self) -> bool:
        """Return whether or not the scene graph has a change."""
        with self._mutex:
            return self._has_change

    @property
    def graph(self) -> DynamicSceneGraph:
        """Return the latest scene graph."""
        with self._mutex:
            self._has_change = False
            return self._graph

    def _poll_thread(self):
        while not self._should_shutdown.is_set():
            if not self._receiver.recv(self._poll_time_ms):
                continue

            with self._mutex:
                self._has_change = True
                graph = self._receiver.graph
                self._graph = graph if graph is None else graph.clone()
