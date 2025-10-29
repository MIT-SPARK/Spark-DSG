"""Entry point for visualizing scene graph."""

import time

import click
import spark_dsg as dsg
from spark_dsg.viser import ViserRenderer


@click.command("visualize")
@click.argument("filepath", type=click.Path(exists=True))
@click.option("--ip", default="localhost")
@click.option("--port", default="8080")
def cli(filepath, ip, port):
    """Visualize a scene graph from FILEPATH using Open3D."""
    G = dsg.DynamicSceneGraph.load(filepath)

    with ViserRenderer(ip, port=port) as renderer:
        renderer.draw(G)
        while True:
            time.sleep(10.0)
