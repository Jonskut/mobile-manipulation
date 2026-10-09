cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include include/common include/nlohmann include/champ)
link_directories(src)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_sdk2/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../../d1_sdk/src/msg")

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  unitree_arm
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_arm REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)
#add_executable(record_bag src/record_bag.cpp)
add_executable(go2_sport_client src/go2/go2_sport_client.cpp src/common/ros2_sport_client.cpp)
target_compile_features(go2_sport_client PRIVATE cxx_std_20)

add_executable(go2_stand_example src/go2/go2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(go2_stand_example PRIVATE cxx_std_20)
add_executable(go2_d1_stand_controller
  src/go2/go2_d1_stand_controller.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../d1_sdk/src/msg/ArmString_.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../d1_sdk/src/msg/PubServoInfo_.cpp"
)
target_compile_features(go2_d1_stand_controller PRIVATE cxx_std_20)
target_link_libraries(go2_d1_stand_controller unitree_sdk2)
add_executable(go2_d1_arm_poses
  src/go2/go2_d1_arm_poses.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../d1_sdk/src/msg/ArmString_.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../d1_sdk/src/msg/PubServoInfo_.cpp"
)
target_compile_features(go2_d1_arm_poses PRIVATE cxx_std_20)
target_link_libraries(go2_d1_arm_poses unitree_sdk2)

add_executable(go2_champ_dds_bridge
  src/go2/go2_champ_dds_bridge.cpp
)
target_compile_features(go2_champ_dds_bridge PRIVATE cxx_std_20)
target_link_libraries(go2_champ_dds_bridge unitree_sdk2)
add_executable(go2_champ_gait_planner
  src/go2/go2_champ_gait_planner.cpp
)
target_compile_features(go2_champ_gait_planner PRIVATE cxx_std_20)

add_executable(go2_robot_state_client src/go2/go2_robot_state_client.cpp src/common/motor_crc.cpp) 
target_compile_features(go2_robot_state_client PRIVATE cxx_std_20)














ament_target_dependencies(low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
#ament_target_dependencies(record_bag ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(go2_stand_example ${DEPENDENCY_LIST})
ament_target_dependencies(go2_d1_stand_controller ${DEPENDENCY_LIST})
ament_target_dependencies(go2_d1_arm_poses ${DEPENDENCY_LIST})
ament_target_dependencies(go2_champ_gait_planner ${DEPENDENCY_LIST})
# Eigen for the payload CoM estimator (header-only FK).
target_include_directories(go2_champ_gait_planner PRIVATE ${EIGEN3_INCLUDE_DIR})
# NOTE: go2_champ_dds_bridge intentionally has NO ament_target_dependencies:
# it must stay ROS-free (Unitree's bundled CycloneDDS clashes with ROS's
# rmw_cyclonedds_cpp inside one process). It talks to the gait planner
# over loopback UDP instead.


ament_target_dependencies(go2_robot_state_client ${DEPENDENCY_LIST})

install(TARGETS 
        low_level_ctrl
        read_low_state
        read_motion_state
        read_wireless_controller
        go2_sport_client
        go2_stand_example
        go2_champ_dds_bridge
        go2_champ_gait_planner
        DESTINATION lib/${PROJECT_NAME}
        )


install(TARGETS go2_robot_state_client)
install(TARGETS go2_d1_stand_controller)
install(TARGETS go2_d1_arm_poses)



install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
