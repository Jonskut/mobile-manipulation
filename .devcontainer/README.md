# Project Dev Container

This container combines the ROS 2 Humble, Unitree SDK2, and MuJoCo development environments used by this repository.

Open the repository root in VS Code and run **Dev Containers: Reopen in Container**.

The image builds `unitree_sdk2` and installs it under `/opt/unitree_robotics`. It downloads and builds MuJoCo 3.3.6 under `/opt/mujoco`, keeping both its source tree and compiled install tree because the simulator uses MuJoCo source files as well as its shared library. The repository is mounted at `/workspace`. The default configuration is headless-safe; GUI and hardware access can be added with a local Compose override when needed.

## Build the C++ simulator

```bash
cd /workspace/unitree_mujoco/simulate
cmake -S . -B build
cmake --build build --parallel
./build/unitree_mujoco -r go2 -s scene_terrain.xml
```

The simulator automatically uses `/opt/mujoco` in the container. For a local non-container build, set `-DMUJOCO_ROOT=/path/to/.mujoco` if the MuJoCo files are not in the workspace default location.

## Build ROS 2 workspaces

```bash
cd /workspace/unitree_ros2/cyclonedds_ws
colcon build

cd /workspace/unitree_ros2/example
colcon build
```

The shell sources ROS 2 Humble and uses Cyclone DDS on the `lo` interface by default. Change `CYCLONEDDS_URI` if connecting to a physical robot through another interface.

## Optional Ubuntu host integration

The default container starts without requiring a host display, GPU, or joystick. To enable MuJoCo's native GUI on an Ubuntu desktop, create `.devcontainer/docker-compose.override.yml` with the following service additions, then reopen the container:

```yaml
services:
	mobile-manipulation:
		environment:
			DISPLAY: ${DISPLAY}
			XAUTHORITY: /tmp/.Xauthority
		volumes:
			- /tmp/.X11-unix:/tmp/.X11-unix:rw
			- ${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro
		devices:
			- /dev/dri:/dev/dri
			- /dev/input:/dev/input
```

The override is intentionally local and should not be committed. Remove the GUI override for headless machines.
