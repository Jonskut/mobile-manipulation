Required ros2 humble and vs code dev containers

Terminal 1

```bash
cd /workspace/unitree_mujoco/simulate
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:${LD_LIBRARY_PATH}"
./build/unitree_mujoco -r go2 -s scene_terrain.xml
```

Terminal 2
```bash
cd /workspace/unitree_ros2/example/src

rm -rf build install log

source /opt/ros/humble/setup.bash
source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

colcon build --packages-select unitree_ros2_example --parallel-workers 1
source install/setup.bash

ros2 run unitree_ros2_example go2_d1_stand_controller
```

