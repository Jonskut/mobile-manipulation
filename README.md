# Mobile manipulation
## Researchers:
Petteri Pyhäluoto, Eero Rannikko, Jonatan Tevaniemi

## Installation
#### Required ros2 humble and vs code dev containers
#### Clone repo
#### Open in dev containers (shift+ctrl+P) -> rebuild and reopen in container

## Run the simulation
### Terminal 1
Build
```bash
cd /workspace/unitree_mujoco/simulate
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```
Run
```bash
cd /workspace/unitree_mujoco/simulate
./build/unitree_mujoco -r go2 -s scene_terrain.xml
```

### Terminal 2 ARM
Build
```bash
cd /workspace/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1
source install/setup.bash
source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash

cd /workspace/unitree_ros2/example
export MAKEFLAGS="-j 1"
colcon build --packages-select unitree_ros2_example --executor sequential
```
#### Arm poses test
```bash
./build/unitree_ros2_example/go2_d1_arm_poses --help
./build/unitree_ros2_example/go2_d1_arm_poses home        # sim (domain 1, lo)
./build/unitree_ros2_example/go2_d1_arm_poses left
./build/unitree_ros2_example/go2_d1_arm_poses demo
./build/unitree_ros2_example/go2_d1_arm_poses forward eth0  # real (domain 0)
```
#### Listen to payload com message
```bash
ros2 topic echo /payload_com
```

### Terminal 3 Champ 
Build
```bash
cd /workspace/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1
source install/setup.bash
source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash

cd /workspace/unitree_ros2/example
export MAKEFLAGS="-j 1"
colcon build --packages-select unitree_ros2_example --executor sequential
```
Run
```bash 
ros2 launch unitree_ros2_example go2_champ_ros2.launch.py dds_domain_id:=1 dds_interface:=lo kp:=60.0 kd:=5.0
```

#### Send twist messages
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```