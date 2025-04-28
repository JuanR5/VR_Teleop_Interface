# Use full ROS 2 Humble desktop image
FROM ros:humble

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Arguments for custom user
ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Set working directory
WORKDIR /tmp

# Install basic dependencies and sudo
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        bash-completion \
        vim \
        python3-pip \
        python3-colcon-common-extensions \
        python3-colcon-argcomplete \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# Install ROS 2 and MoveIt related dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-ament-flake8 \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-joint-trajectory-controller \
        ros-humble-rviz2 \
        ros-humble-xacro \
        libeigen3-dev \
        libignition-gazebo6-dev \
        libpoco-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python linters and tools
RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes

# Build and install libfranka manually
WORKDIR /home/$USERNAME/source_code
RUN git clone https://github.com/frankaemika/libfranka.git && \
    cd libfranka && \
    git checkout 0.13.6 && \
    git submodule update --init && \
    mkdir build && cd build && \
    cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF .. && \
    make franka -j$(nproc) && \
    sudo cpack -G DEB && \
    sudo dpkg -i libfranka*.deb

# Setup ROS2 workspace
WORKDIR /home/$USERNAME/ros2_ws

# Clone your VR Teleoperation Interface
RUN mkdir -p tmp_clone
WORKDIR /home/$USERNAME/ros2_ws/tmp_clone
RUN git clone --single-branch --branch cubi https://github.com/JuanR5/VR_Teleop_Interface.git .

# Move ROS2 packages to src/
WORKDIR /home/$USERNAME/ros2_ws
RUN mkdir -p src && \
    mv tmp_clone/src/* src/ && \
    rm -rf tmp_clone

# Set permissions
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME

# Switch to non-root user
USER $USERNAME

# Set default shell
SHELL ["/bin/bash", "-c"]

# Automatically source, build workspace, and drop to shell
CMD source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd ~/ros2_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash && \
    exec bash
