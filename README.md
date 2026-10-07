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

Terminal 2 ARM
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
ros2 run unitree_ros2_example go2_d1_stand_controller
```

The controller uses the native Unitree DDS topic names `rt/lowcmd`,
`rt/lowstate`, and `rt/arm_Command`, so the same command and feedback topics
are used with MuJoCo and the physical robot.

Terminal 3 Champ 
```bash 
cd /workspace/unitree_ros2/example
export MAKEFLAGS="-j 1"
colcon build --packages-select unitree_ros2_example --executor sequential

source /opt/ros/humble/setup.bash
source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash
source /workspace/unitree_ros2/example/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

ros2 launch unitree_ros2_example go2_champ_walk.launch.py
# or: 
ros2 launch unitree_ros2_example go2_champ_walk.launch.py dds_domain_id:=1 dds_interface:=lo kp:=60.0 kd:=5.0
```

Send twist messages
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```