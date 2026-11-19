# Jetson Orin, JetPack 5.x (L4T 35.x, Ubuntu 20.04, CUDA 11.4), ROS 2 Humble.
# Humble debs do not exist for Ubuntu 20.04, so the base image provides Humble.
# NOT YET BUILT ON THE ORIN: tags/URLs below are the best known values; see VERIFY.md.
ARG BASE=dustynv/ros:humble-ros-base-l4t-r35.4.1
FROM ${BASE}
ARG ZED_SDK_URL=https://download.stereolabs.com/zedsdk/4.2/l4t35.4/jetsons
ARG ZED_WRAPPER_REF=humble-v4.2.x
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      zstd udev lsb-release wget git cmake build-essential python3-pip python3-yaml \
      libusb-1.0-0-dev libpcl-dev libapr1-dev && rm -rf /var/lib/apt/lists/*

# ZED SDK. Must match the exact L4T minor version (ZED_SDK_URL build arg).
RUN wget -q -O /tmp/zed.run "${ZED_SDK_URL}" && chmod +x /tmp/zed.run && \
    /tmp/zed.run -- silent skip_od_module skip_python skip_drivers && rm /tmp/zed.run

# CuPy for CUDA 11.x. Python 3.8 is the system Python on Ubuntu 20.04.
RUN pip3 install --no-cache-dir "cupy-cuda11x" numpy pyyaml

# Livox SDK2 (the driver links against it)
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 && \
    cd /opt/Livox-SDK2 && mkdir build && cd build && cmake .. && make -j$(nproc) && make install

WORKDIR /ws
RUN mkdir -p src && cd src && \
    git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git && \
    cp livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml && \
    git clone --recursive --depth 1 -b "${ZED_WRAPPER_REF}" https://github.com/stereolabs/zed-ros2-wrapper.git

COPY src/elev_bringup /ws/src/elev_bringup

# Install every ROS dependency that apt can provide for the workspace, then build.
RUN source /opt/ros/humble/setup.bash && \
    (rosdep update || true) && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys "zed_camera" && \
    colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
