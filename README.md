## Kimera-DSG

This is the core library that contains the dynamic scene graph data-structure. If you're looking at creating the DSG, you probably want [this](https://github.mit.edu/SPARK/Kimera-DSG-Builder) repo instead.

This repo contains:
  * `kimera_dsg`: core C++ library and (partial) python bindings
  * `kimera_dsg_visualizer`: RViz based visualization for a DSG

### Building for Python

  1. Install requirements and make a virtual environment:

```bash
sudo apt install libpcl-dev libgoogle-glog-dev python3-venv
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

git clone git@github.mit.edu:SPARK/Kimera-DSG.git
cd Kimera-DSG
pip install -e .
```

   You can also directly install from github if you don't need a development install, like so:
```
# you might want to set cmake to use multiple threads
# export CMAKE_BUILD_PARALLEL_LEVEL=8

# the branch you want to install is after the second @ sign
pip install git+ssh://git@github.mit.edu:SPARK/Kimera-DSG.git@feature/pip_package
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

### Obtaining and Building For ROS

Requirements (you likely have them):

```bash
sudo apt install python-rosdep python-wstool python-catkin-tools
# if you haven't yet:
# sudo rosdep init
# rosdep update
```

To get started (YMMV):

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init

cd src
git clone git@github.mit.edu:SPARK/Kimera-DSG.git kimera_dsg
cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin build
```

File any and all build errors as an issue
