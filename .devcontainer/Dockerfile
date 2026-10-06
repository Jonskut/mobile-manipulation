FROM althack/ros2:humble-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV CC=/usr/bin/gcc
ENV CXX=/usr/bin/g++
ENV PATH=/opt/venv/bin:${PATH}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=1
ENV CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime \
    && echo ${TZ} > /etc/timezone \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        gdb \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libglfw3-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        ninja-build \
        python3-pip \
        python3-venv \
        zsh \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

COPY unitree_sdk2 /tmp/unitree_sdk2
RUN rm -rf /tmp/unitree_sdk2/build \
    && cmake -S /tmp/unitree_sdk2 -B /tmp/unitree_sdk2/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics \
        -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/unitree_sdk2/build --parallel \
    && cmake --install /tmp/unitree_sdk2/build \
    && mkdir -p /opt/unitree_robotics/include \
    && cp -a /tmp/unitree_sdk2/include/. /opt/unitree_robotics/include/ \
    && rm -rf /tmp/unitree_sdk2

RUN git clone --depth 1 --branch 3.3.6 \
        https://github.com/google-deepmind/mujoco.git \
        /opt/mujoco/mujoco-3.3.6 \
    && cmake -S /opt/mujoco/mujoco-3.3.6 -B /tmp/mujoco-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/mujoco/install-mujoco-3.3.6 \
        -DMUJOCO_BUILD_TESTS=OFF \
        -DMUJOCO_BUILD_EXAMPLES=OFF \
        -DMUJOCO_BUILD_SIMULATE=ON \
        -DMUJOCO_WITH_USD=OFF \
    && cmake --build /tmp/mujoco-build --parallel \
    && cmake --install /tmp/mujoco-build \
    && rm -rf /tmp/mujoco-build /opt/mujoco/mujoco-3.3.6/.git

RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir \
        catkin_pkg \
        empy==3.3.4 \
        lark==1.1.9 \
        mujoco \
        pygame

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'export CMAKE_PREFIX_PATH="/opt/unitree_robotics:${CMAKE_PREFIX_PATH}"' \
    'export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:${LD_LIBRARY_PATH}"' \
    'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}"' \
    'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"' \
    'export CYCLONEDDS_URI="${CYCLONEDDS_URI}"' \
    'if [ -f /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash ]; then source /workspace/unitree_ros2/cyclonedds_ws/install/setup.bash; fi' \
    'if [ -f /workspace/unitree_ros2/example/install/setup.bash ]; then source /workspace/unitree_ros2/example/install/setup.bash; fi' \
    > /etc/profile.d/mobile-manipulation.sh

RUN echo 'source /etc/profile.d/mobile-manipulation.sh' >> /root/.bashrc \
    && echo 'source /etc/profile.d/mobile-manipulation.sh' >> /root/.zshrc

WORKDIR /workspace
CMD ["bash"]
