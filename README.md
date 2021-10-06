## Kimera-DSG

This is the core library that contains the dynamic scene graph data-structure. If you're looking at creating the DSG, you probably want [this](https://github.mit.edu/SPARK/Kimera-DSG-Builder) repo instead.

This repo contains:
  * `kimera_dsg`: core C++ library
  * `kimera_dsg_python`: python bindings and native python helpers
  * `kimera_dsg_visualizer`: RViz based visualization for a DSG

### Obtaining and Building

Requirements (you likely have them):

```
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
git clone git@github.mit.edu:SPARK/Kimera-DSG.git kimera_dsg --recursive
cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin build
```

File any and all build errors as an issue

### API documentation

Both the c++ core code and the python bindings have API documentation that can be generated. Run

```
./make_docs.py
```

from the root directory of this repo, and point your browser to the URL shown (the script makes a bunch of html files in `/tmp/kimera_dsg_docs` and then servers them via `python3 -m http.server`).

:warning: The API documentation is very preliminary (in particular, the python bindings are currently scraped from the extension modules and made into a fake pure-python module for Sphinx to view, which is brittle) and the formatting may be extremely tough to read.

### Usage

Code to build the scene graph lives [here](https://github.mit.edu/SPARK/Kimera-DSG-Builder)
