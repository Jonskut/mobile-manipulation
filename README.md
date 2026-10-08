Required ros2 humble and vs code dev containers

Open in dev containers (shift+ctrl+P) -> rebuild and reopen in container

Terminal 1

```bash
cd /workspace/unitree_mujoco/simulate
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

cd /workspace/unitree_mujoco/simulate
./build/unitree_mujoco -r go2 -s scene_terrain.xml
```

Terminal 2
```bash
cd /workspace/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1
source install/setup.bash
source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash

cd /workspace/unitree_ros2/example
export MAKEFLAGS="-j 1"
colcon build --packages-select unitree_ros2_example --executor sequential


cd /workspace/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /workspace/unitree_ros2/example
source install/setup.bash
ros2 run unitree_ros2_example go2_stand_example
```

The controller uses the native Unitree DDS topic names `rt/lowcmd`,
`rt/lowstate`, and `rt/arm_Command`, so the same command and feedback topics
are used with MuJoCo and the physical robot.

Champ 
```bash 
cd /workspace/unitree_ros2/example
export MAKEFLAGS="-j 1"
colcon build --packages-select unitree_ros2_example --executor sequential

source /opt/ros/humble/setup.bash
source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash
source /workspace/unitree_ros2/example/install/setup.bash

ros2 launch unitree_ros2_example go2_arm_example
```