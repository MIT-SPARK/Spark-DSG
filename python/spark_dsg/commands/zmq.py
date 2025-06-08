"""Entry point for sending and receiving the scene graph by zmq."""

import time

import click
import spark_dsg as dsg
from spark_dsg.zmq import DsgReceiver, DsgSender, ZmqGraph


@click.group(name="zmq")
def cli():
    """Contains commands to send and receive a scene graph by zmq."""
    pass


@cli.command()
@click.argument("filepath", type=click.Path(exists=True))
@click.option("--url", "-u", default="tcp://127.0.0.1:8001")
@click.option("--rate", "-r", default=1.0, help="publish rate", type=float)
def send(filepath, url, rate):
    """Load a scene graph from file and send at a fix rate."""
    sender = DsgSender(url)
    G = dsg.DynamicSceneGraph.load(filepath)

    while True:
        sender.send(G)
        time.sleep(rate)


@cli.command()
@click.argument("url", type=str)
@click.option("--wait-duration-ms", "-w", type=int, default=1000)
def echo(url, wait_duration_ms):
    """Receive a scene graph via zmq and print the number of nodes received."""
    receiver = DsgReceiver(url)

    while True:
        if not receiver.recv(10):
            print("No graph received!")
            time.sleep(wait_duration_ms / 1000.0)
            continue

        print(f"Got graph with {receiver.graph.num_nodes()} nodes!")
        time.sleep(wait_duration_ms / 1000.0)


@cli.command()
@click.argument("url", type=str)
@click.option("--poll-time-ms", "-w", type=int, default=100)
def monitor(url, poll_time_ms):
    """Use ZmqGraph to monitor an url for changes."""
    with ZmqGraph(url) as zmq_graph:
        num_nodes = None
        while True:
            have_change = False
            if zmq_graph.has_change and zmq_graph.graph is not None:
                have_change = True
                num_nodes = zmq_graph.graph.num_nodes()

            nodes_str = f"{num_nodes}" if num_nodes else "N/A"
            change_str = "yes" if have_change else "no"
            print(f"Graph has {nodes_str} nodes (changed: {change_str})")
            time.sleep(poll_time_ms / 1000.0)
