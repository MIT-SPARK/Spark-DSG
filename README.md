## Spark-DSG

This is the core c++ library that contains the dynamic scene graph data-structure used by Hydra. It also has python bindings.

### Acknowledgements and Disclaimer

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

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

git clone git@github.com:MIT-SPARK/Spark-DSG.git
cd Spark-DSG
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
