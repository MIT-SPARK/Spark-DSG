"""Entry points for hydra."""
import click
import spark_dsg.commands.visualize as visualize


@click.group()
def cli():
    """Entry point target for subcommands."""
    pass


cli.add_command(visualize.cli)
