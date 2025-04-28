# Set build arguments
ARG UBUNTU_MAJOR=22
ARG UBUNTU_MINOR=04
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=6
ARG CUDA_PATCH=3
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=2
ARG ZED_SDK_PATCH=5
ARG IMAGE_NAME=nvcr.io/nvidia/cuda:${CUDA_MAJOR}.${CUDA_MINOR}.${CUDA_PATCH}-devel-ubuntu${UBUNTU_MAJOR}.${UBUNTU_MINOR}

FROM ${IMAGE_NAME}

# Inherit and set environment variables
ARG UBUNTU_MAJOR
ARG UBUNTU_MINOR
ARG CUDA_MAJOR
ARG CUDA_MINOR
ARG CUDA_PATCH
ARG ZED_SDK_MAJOR
ARG ZED_SDK_MINOR
ARG ZED_SDK_PATCH
ARG CUSTOM_ZED_SDK_URL=""
ARG ROS2_DIST=humble

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility
ENV ZED_SDK_URL=${CUSTOM_ZED_SDK_URL:-"https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_MAJOR}"}
ENV TZ=Europe/Paris
ENV ROS_DISTRO=${ROS2_DIST}

# Install core dependencies
RUN apt-get update && \
  apt-get install -y --no-install-recommends \
  tzdata apt-utils dialog curl wget sudo gnupg2 software-properties-common \
  lsb-release build-essential cmake git python3 python3-dev python3-pip python3-wheel \
  libopencv-dev libpq-dev libusb-1.0-0-dev usbutils zstd jq less udev bash-completion && \
  ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
  rm -rf /var/lib/apt/lists/*

# Validate ZED SDK URL
RUN echo "Validating SDK URL: $ZED_SDK_URL" && \
  if [ "$(curl -L -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n')" != "200" ]; then \
    echo "ZED SDK URL is invalid." && exit 1; \
  fi

# Set up locale
RUN apt-get update && \
  apt-get install -y locales && \
  locale-gen en_US.UTF-8 && \
  update-locale LANG=en_US.UTF-8 && \
  rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

############ Install ROS2 ############

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
  apt-get update && \
  apt-get install -y --no-install-recommends \
  ros-${ROS2_DIST}-ros-base \
  python3-flake8-docstrings python3-pytest-cov ros-dev-tools && \
  pip3 install -U argcomplete numpy empy lark && \
  rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

############ Install ZED SDK ############

RUN echo "Installing ZED SDK..." && \
  wget -q -O ZED_SDK_Linux_Ubuntu.run "${ZED_SDK_URL}" && \
  chmod +x ZED_SDK_Linux_Ubuntu.run && \
  ./ZED_SDK_Linux_Ubuntu.run -- silent skip_tools skip_cuda && \
  ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
  rm ZED_SDK_Linux_Ubuntu.run && \
  echo "CUDA Version $CUDA_MAJOR.$CUDA_MINOR.$CUDA_PATCH" > /usr/local/cuda/version.txt && \
  rm -rf /var/lib/apt/lists/*

############ Build ROS2 Workspace ############

# Clone the repository
WORKDIR /root/ros2_ws/tmp_clone
RUN git clone --single-branch --branch aorus_zed https://github.com/JuanR5/VR_Teleop_Interface.git . #!

# Move only the ROS 2 packages into src/
WORKDIR /root/ros2_ws/
RUN mkdir -p src && \
    mv tmp_clone/src/* src/ && \
    rm -rf tmp_clone

RUN bash -c "source /opt/ros/${ROS2_DIST}/setup.bash && \
  apt-get update && rosdep update && \
  rosdep install --from-paths /root/ros2_ws/src --ignore-src -r -y && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
  -DCMAKE_CXX_FLAGS='-Wl,--allow-shlib-undefined'" && \
  rm -rf /var/lib/apt/lists/*

# Entrypoint setup
RUN echo '#!/bin/bash\n\
source "/opt/ros/$ROS_DISTRO/setup.bash"\n\
source "/root/ros2_ws/install/setup.bash"\n\

echo -e "\n🚀 To launch the ZED camera:\n  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=cameraModel"\n\
echo -e "\n🎮 For VR integration (ROS TCP + Unity stereo image + ZED):\n  ros2 launch middle_nodes zed_vr_conexion.launch.py\n"\n\

exec "$@"' > /sbin/ros_entrypoint.sh && \
    chmod +x /sbin/ros_entrypoint.sh

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]
