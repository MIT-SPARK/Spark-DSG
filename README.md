## Kimera-DSG

This is the core library that contains the dynamic scene graph data-structure. If you're looking at creating or  visualizing the DSG, you probably want [this](https://github.mit.edu/SPARK/Kimera-DSG-Builder) repo instead.

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
git clone git@github.mit.edu:SPARK/Kimera-DSG.git kimera_dsg
cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin build
```

File any and all build errors as an issue

### Usage

Visualization and builder code [here](https://github.mit.edu/SPARK/Kimera-DSG-Builder)
