"""Entry points for spark_dsg."""

import click

import spark_dsg.commands.visualize as visualize
import spark_dsg.commands.zmq as zmq


@click.group()
def cli():
    """Entry point target for subcommands."""
    pass


cli.add_command(visualize.cli)
cli.add_command(zmq.cli)

if __name__ == "__main__":
    cli()
