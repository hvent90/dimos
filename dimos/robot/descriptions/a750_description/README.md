# ROS2 support and URDF descriptions for A-750 robotic arm


## Installation

To use this package, make sure you have: [ROS2 installed](https://docs.ros.org/en/jazzy/Installation.html)
Then set up the workspace:

```bash
# Set up environment variables if you haven't already (customize as needed)
export ROS_DISTRO=jazzy  # Change to your ROS2 distro (humble, iron, jazzy, etc.)
export ROS_WS=~/ros2_ws   # Customize workspace path

# Head to the workspace
cd $ROS_WS/src

# Source your ros2 distro
source /opt/ros/$ROS_DISTRO/setup.bash

# Clone the package
git clone https://github.com/adob/a750_ros2.git

# Build the workspace
cd $ROS_WS
colcon build

# Source the workspace
source $ROS_WS/install/setup.bash
```

## Universal Robot Description Files (URDF)


## Universal Robot Description Files (URDF)

Kinematic and related parameters are defined in URDF files.

### Visualization
To display the robot in RViz with a simple joint state GUI:

```bash
ros2 launch a750_description visualize_a750.launch.py hwrev:=1
```
