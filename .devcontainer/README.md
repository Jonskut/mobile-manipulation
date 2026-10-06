# Project Dev Container

This container combines the ROS 2 Humble, Unitree SDK2, and MuJoCo development environments used by this repository.

Open the repository root in VS Code and run **Dev Containers: Reopen in Container**.

The image builds `unitree_sdk2` and installs it under `/opt/unitree_robotics`. The repository is mounted at `/workspace`, and the container passes through the X11 socket, `/dev/dri`, and joystick devices for MuJoCo and ROS 2 simulation.

## Build the C++ simulator

```bash
cd /workspace/unitree_mujoco/simulate
cmake -S . -B build
cmake --build build --parallel
./build/unitree_mujoco -r go2 -s scene_terrain.xml
```

## Build ROS 2 workspaces

```bash
cd /workspace/unitree_ros2/cyclonedds_ws
colcon build

cd /workspace/unitree_ros2/example
colcon build
```

The shell sources ROS 2 Humble and uses Cyclone DDS on the `lo` interface by default. Change `CYCLONEDDS_URI` if connecting to a physical robot through another interface.
