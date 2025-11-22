# PDE4430
#### Middlesex University Dubai

## Turtlesim Assessment
 TurtleSim Motion Nodes - ROS2

This ROS2 package contains Python nodes that simulate different motion patterns in TurtleSim, including a **Roomba-like cleaning behavior**, a **figure-8 path**, and a **circular path**.

---

## Package Contents

- **scripts/roomba.py**: Simulates a Roomba vacuum. Moves forward and turns when reaching walls to cover the TurtleSim frame.
- **scripts/Figure8.py**: Moves the turtle in a figure-8 pattern using alternating clockwise and counter-clockwise rotations.
- **scripts/circle.py**: Moves the turtle in a continuous circular path with a fixed radius.
- **turtlesim_pde4430_HamidaSajjad/**: Python module folder (contains `__init__.py`).
- **setup.py** and **package.xml**: ROS2 package configuration files.

---

## Prerequisites

- ROS2 Jazzy installed
- `turtlesim` package installed
- Python 3

---

## Installation & Build

1. Clone the repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <git@github.com:hamidasajjad-boop/turtlesim_pde4430_HamidaSajjad.git>

    Make the Python scripts executable:

cd turtlesim_pde4430_HamidaSajjad/turtlesim_pde4430_HamidaSajjad
chmod +x *.py

    Build the workspace:

cd ~/ros2_ws
colcon build

    Source the workspace:

source install/setup.bash

Usage

    Start the TurtleSim node:

ros2 run turtlesim turtlesim_node

    In another terminal, run any of the motion nodes:

ros2 run turtlesim_pde4430_HamidaSajjad roomba
ros2 run turtlesim_pde4430_HamidaSajjad circle
ros2 run turtlesim_pde4430_HamidaSajjad Figure8

Node Behavior
Roomba Node (roomba.py)

    Moves forward continuously.

    Turns when approaching walls to cover the full area.

    Simulates a robot vacuum cleaning behavior.

Figure8 Node (Figure*.py)

    Moves in a figure-8 path.

    Alternates angular direction every full circle to create the figure-8.

Circle Node (circle.py)

    Moves in a constant circular path.

    Linear and angular speeds are set to maintain a fixed radius.

Notes

    Python nodes do not require rebuilding after edits; just restart the node.

    Parameters like speed, radius, and turning behavior can be adjusted directly in the scripts.

    Make sure all scripts in scripts/ are executable.

Author

Hamida Sajjad
ROS2 TurtleSim Motion Nodes Assignment
