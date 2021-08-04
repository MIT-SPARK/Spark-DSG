#!/usr/bin/env python3
"""Build docs for the package."""
import subprocess
import argparse
import tempfile
import pathlib
import logging
import sys
import os


def setup_venv(dirpath):
    """Make a virtual environment for building the documentation."""
    ret = subprocess.run(
        ["python3", "-m", "venv", "-h"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if ret.returncode != 0:
        install_command = "sudo apt install python3-venv"
        logging.fatal(
            "venv not found. You can install it via {}.".format(install_command)
        )
        sys.exit(1)

    subprocess.run(["python3", "-m", "venv", "kimera_dsg_docs_env"], cwd=str(dirpath))

    env_python_dir = dirpath / "kimera_dsg_docs_env"
    env_python_bin = env_python_dir / "bin" / "python"
    subprocess.run([str(env_python_bin), "-m", "pip", "install", "--upgrade", "pip"])
    requirements = [
        "sphinx<4.1",
        "sphinxcontrib-napoleon",
        "breathe",
        "exhale",
        "sphinx-rtd-theme",
    ]
    for requirement in requirements:
        subprocess.run([str(env_python_bin), "-m", "pip", "install", requirement])

    return env_python_dir


def make_docs(env_python_dir, output_path):
    """Make the documentation."""
    curr_file = pathlib.Path(__file__).absolute()
    doc_dir = curr_file.parent / "doc"
    script_dir = curr_file.parent / "kimera_dsg_python" / "scripts"
    script_file = script_dir / "scrape_docstrings.py"

    python_version_str = "python{}.{}".format(
        sys.version_info.major, sys.version_info.minor
    )
    env_lib_dir = env_python_dir / "lib" / python_version_str / "site-packages"

    fake_module_output = env_lib_dir / "kimera_dsg_python_bindings.py"
    subprocess.run(["python2", str(script_file), fake_module_output])

    if not output_path.exists():
        output_path.mkdir()

    sphinx_env = os.environ.copy()
    sphinx_env["PYTHONPATH"] = str(env_lib_dir)
    env_python_bin = env_python_dir / "bin" / "python"
    subprocess.run(
        [
            str(env_python_bin),
            "-m",
            "sphinx",
            doc_dir,
            "-b",
            "html",
            "-a",
            str(output_path),
        ], env=sphinx_env
    )


def host_docs(output_path, port=None, address=None):
    """Start up simple web server."""
    args = ["python3", "-m", "http.server"]
    if address is not None:
        args += ["-b", address]

    if port is not None:
        args += [str(port)]

    logging.info("hosting docs at http://0.0.0.0:8000")
    subprocess.run(args, cwd=str(output_path))


def main():
    """Set up the build process."""
    parser = argparse.ArgumentParser(
        description="utility to build and (optionally) serve docs locally"
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        type=str,
        help="docs output dir (defaults to /tmp/kimera_dsg_docs",
    )
    args = parser.parse_args()

    if args.output is None:
        output_path = pathlib.Path("/tmp/kimera_dsg_docs")
    else:
        output_path = pathlib.Path(args.output_path)

    logging.basicConfig(level=logging.INFO)
    with tempfile.TemporaryDirectory() as venv_dir:
        venv_path = setup_venv(pathlib.Path(venv_dir))
        make_docs(venv_path, output_path)

    host_docs(output_path)


if __name__ == "__main__":
    main()
