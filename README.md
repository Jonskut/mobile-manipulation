Required ros2 humble and vs code dev containers

Open in dev containers (shift+ctrl+P) -> rebuild and reopen in container

Terminal 1

```bash
cd /workspace/unitree_mujoco/simulate
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/unitree_mujoco -r go2 -s scene_terrain.xml
```

Terminal 2
```bash
cd /workspace/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1
source install/setup.bash

cd /workspace/unitree_ros2/example/src

rm -rf build install log

source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash

export MAKEFLAGS="-j 1"
colcon build --packages-select unitree_ros2_example --executor sequential
source install/setup.bash

ros2 run unitree_ros2_example go2_d1_stand_controller
```

The controller uses the native Unitree DDS topic names `rt/lowcmd`,
`rt/lowstate`, and `rt/arm_Command`, so the same command and feedback topics
are used with MuJoCo and the physical robot.

Champ 
```bash 
cd /workspaces/mobile-manipulation/unitree_ros2/example  # per README layout
colcon build --packages-select unitree_ros2_example
source install/setup.bash
ros2 launch unitree_ros2_example go2_champ_walk.launch.py
```