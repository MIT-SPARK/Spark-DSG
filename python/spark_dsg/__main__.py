"""Entry points for spark_dsg."""

import click
import pathlib

import spark_dsg as dsg
import spark_dsg.commands.visualize as visualize
import spark_dsg.commands.zmq as zmq


@click.group()
def cli():
    """Entry point target for subcommands."""
    pass


cli.add_command(visualize.cli)
cli.add_command(zmq.cli)


@cli.command()
@click.argument("filepaths", nargs=-1, type=click.Path(exists=True))
def update(filepaths):
    """Update a collection of files."""
    for filepath in filepaths:
        filepath = pathlib.Path(filepath).expanduser().absolute()
        try:
            G = dsg.SceneGraph.load(filepath)
            G.save(filepath, include_mesh=True)
        except Exception as e:
            click.secho(f"Warning: failed to convert '{filepath}': {e}")


if __name__ == "__main__":
    cli()
