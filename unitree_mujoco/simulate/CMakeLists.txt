cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

set(MUJOCO_ROOT "" CACHE PATH "MuJoCo root containing source and install trees")
if (NOT MUJOCO_ROOT)
  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../.mujoco")
    set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.mujoco")
  elseif (EXISTS "/opt/mujoco")
    set(MUJOCO_ROOT "/opt/mujoco")
  else()
    message(FATAL_ERROR "MuJoCo was not found. Set MUJOCO_ROOT to a directory containing mujoco-3.3.6 and install-mujoco-3.3.6.")
  endif()
endif()

set(MUJOCO_SOURCE_DIR "${MUJOCO_ROOT}/mujoco-3.3.6")
set(MUJOCO_INSTALL_DIR "${MUJOCO_ROOT}/install-mujoco-3.3.6")
if (NOT EXISTS "${MUJOCO_SOURCE_DIR}/simulate/simulate.cc")
  message(FATAL_ERROR "MuJoCo source tree is missing: ${MUJOCO_SOURCE_DIR}")
endif()
if (NOT EXISTS "${MUJOCO_INSTALL_DIR}/lib/libmujoco.so")
  message(FATAL_ERROR "MuJoCo install tree is missing: ${MUJOCO_INSTALL_DIR}")
endif()

set(UNITREE_SDK_SOURCE_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_sdk2/include")
if (EXISTS "${UNITREE_SDK_SOURCE_INCLUDE}/unitree/common/log/log.hpp")
  include_directories("${UNITREE_SDK_SOURCE_INCLUDE}")
endif()

set(D1_SDK_MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../d1_sdk/src/msg")
if (EXISTS "${D1_SDK_MSG_DIR}/ArmString_.hpp")
  include_directories("${D1_SDK_MSG_DIR}")
else()
  message(FATAL_ERROR "D1 SDK message definition not found: ${D1_SDK_MSG_DIR}/ArmString_.hpp")
endif()

include_directories(
  /usr/local/include/iceoryx/v2.0.2
  "${MUJOCO_SOURCE_DIR}/include"
  "${MUJOCO_SOURCE_DIR}/simulate"
  src/lodepng
)
link_directories("${MUJOCO_INSTALL_DIR}/lib")
set(CMAKE_BUILD_RPATH "${MUJOCO_INSTALL_DIR}/lib;${CMAKE_BUILD_RPATH}")

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  "${MUJOCO_SOURCE_DIR}/simulate/glfw_*.cc"
  "${MUJOCO_SOURCE_DIR}/simulate/platform_*.cc"
  "${MUJOCO_SOURCE_DIR}/simulate/simulate.cc"
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco
  ${SIM_SRC}
  src/main.cc
  "${D1_SDK_MSG_DIR}/ArmString_.cpp"
  "${D1_SDK_MSG_DIR}/PubServoInfo_.cpp"
)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
