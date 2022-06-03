## Spark-DSG

This is the core c++ library that contains the dynamic scene graph data-structure used by Hydra. It also has python bindings.

### Building for Python

  1. Install requirements and make a virtual environment:

```bash
sudo apt install libpcl-dev python3-venv
mkdir /path/to/environment
cd /path/to/environment
python3 -m venv dsg  # or some other environment name

# you may also want to upgrade pip on 18.04, though it shouldn't be necessary
# source dsg/bin/activate
# pip install --upgrade pip
```

  2. Install the python package
```bash
source /path/to/dsg/environment/bin/activate

# you might want to set cmake to use multiple threads
# export CMAKE_BUILD_PARALLEL_LEVEL=8

git clone git@github.mit.edu:SPARK/Kimera-DSG.git
cd Kimera-DSG
pip install -e .
```

### Python Bindings Usage

See [this notebook](notebooks/bindings_demo.py) for some examples for the bindings (you'll want to clone the repo, even if you installed from github).  You'll want to install `jupyter` and `jupytext` if you want to run it as a notebook, though you can also just run it directly as a python script.

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

This repository is a valid catkin package and should build if placed in a workspace.
