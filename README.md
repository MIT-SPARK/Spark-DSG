[![Spark-DSG Build and Test](https://github.com/MIT-SPARK/Spark-DSG/actions/workflows/ci.yaml/badge.svg)](https://github.com/MIT-SPARK/Spark-DSG/actions/workflows/ci.yaml)

## Spark-DSG

This is the core c++ library that contains the 3D Scene Graph data-structure used by Hydra. It also has python bindings.

### Change Notes

- 4/22/24: Python bindings and unit tests are built by default. You will need to install relevant dependencies (via `sudo apt install python3-dev` or `rosdep`) or disable the bindings and tests via cmake options.
- 3/16/24: Updated dependencies to use system libraries for `nlohmann_json`. You will need to install it either via `sudo apt install nlohmann-json3-dev` or use `rosdep` to update dependencies.

### Acknowledgements and Disclaimer

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

### Building for Python

  1. Install requirements and make a virtual environment:

```bash
sudo apt install python3-venv libzmqpp-dev nlohmann-json3-dev
mkdir /path/to/environment
cd /path/to/environment
python3 -m venv dsg  # or some other environment name

# you may also want to upgrade pip, though it shouldn't be necessary
# source dsg/bin/activate
# pip install --upgrade pip
```

  2. Install the python package
```bash
source /path/to/dsg/environment/bin/activate
git clone git@github.com:MIT-SPARK/Spark-DSG.git
pip install ./Spark-DSG
```

### Python Bindings Usage

See [this notebook](examples/python_api.py) for some examples for the bindings (you'll want to clone the repo, even if you installed from github).
You'll want to install `jupyter` and `jupytext` if you want to run it as a notebook, though you can also just run it directly as a python script.
You can find an example scene graph [here](https://drive.google.com/file/d/1jwcjrE4-6PvOgEgJipETkQaLgC43biFT/view?usp=sharing).

### Python API documentation

Generating the python documentation should be as simple as:

```
source /path/to/dsg/environment/bin/activate
cd doc
pip install sphinx  # if you haven't already
make html
python -m http.server  # to serve them locally
```

### Building For ROS

This repository is a valid ROS package and should build if placed in a workspace.

### Loading old scene graph files

As of version `1.2.0` of this package, serialization support has been dropped for versions older than `1.1.2`, and files saved in this serialization format will not be loaded.
You can use these files by upgrading the serialization format with an older version of this package.
The most straigthforward way to do this is to install version `1.1.2` or `1.1.3` from PyPi into a virtual environment and manually upgrade.
This looks like:
```
python3 -m virtualenv /tmp/spark_dsg_upgrade --download
source /tmp/spark_dsg_upgrade/bin/actviate
pip install spark_dsg==1.1.3
```
Then in a python REPL or script:
```python
import spark_dsg as dsg
G = dsg.DynamicSceneGraph.load("/path/to/file/to/upgrade")
G.save("/path/to/file/to/upgrade", include_mesh=True)
```
Newer versions on develop (`1.1.5` and onwards) have a commandline tool that allows you to do this for multiple files.
