"""Entry points for spark_dsg."""

import pathlib

import click

import spark_dsg as dsg
from spark_dsg.commands import visualize, zmq


@click.group()
def cli():
    """Entry point target for subcommands."""


cli.add_command(visualize.cli)
cli.add_command(zmq.cli)


@cli.command()
@click.argument("filepaths", nargs=-1, type=click.Path(exists=True))
def update(filepaths):
    """Update a collection of files."""
    dsg.enable_short_serialization_message()
    for filepath in filepaths:
        filepath = pathlib.Path(filepath).expanduser().absolute()
        try:
            G = dsg.SceneGraph.load(filepath)
            G.save(filepath, include_mesh=True)
        except RuntimeError as e:
            click.secho(f"Warning: failed to convert '{filepath}': {e}")


if __name__ == "__main__":
    cli()
