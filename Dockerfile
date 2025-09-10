# ============================================================
# Dockerfile for POLARIS_GEM_e2 Assignment
# Base: Ubuntu 20.04 + ROS Noetic
# ============================================================
FROM ros:noetic-ros-base

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# ------------------------------------------------------------
# STEP 1: Use default Ubuntu mirrors (DO NOT USE ports.ubuntu.com)
# ------------------------------------------------------------
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|mirror://mirrors.ubuntu.com/mirrors.txt|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu/|mirror://mirrors.ubuntu.com/mirrors.txt|g' /etc/apt/sources.list

# ------------------------------------------------------------
# STEP 2: Add GPG keys for ROS
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y curl gnupg2 ca-certificates \
 && curl -sSL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" | apt-key add - \
 && apt-get update

# ------------------------------------------------------------
# STEP 3: Enable universe repository
# ------------------------------------------------------------
RUN sed -Ei 's/^# deb/deb/' /etc/apt/sources.list && apt-get update

# ------------------------------------------------------------
# STEP 4: Install core system dependencies
# ------------------------------------------------------------
RUN apt-get install -y --no-install-recommends \
    sudo \
    wget \
    git \
    lsb-release \
    build-essential \
    cmake \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-colcon-common-extensions \
    libopencv-dev \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    pkg-config \
    apt-transport-https \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# STEP 5: Upgrade Python tools
# ------------------------------------------------------------
RUN apt-get update && apt-get install --reinstall -y python3-pip python3-setuptools python3-wheel \
 && python3 -m pip install --upgrade pip setuptools wheel importlib-metadata

# ------------------------------------------------------------
# STEP 6: Install ROS-specific dependencies
# ------------------------------------------------------------
# Ensure retries for apt-get
RUN echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries

RUN apt-get update && apt-get install -y --fix-missing \
    ros-noetic-cv-bridge \
    ros-noetic-vision-opencv \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator \
 && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# STEP 7: rosdep initialization
# ------------------------------------------------------------
RUN rosdep init || true \
 && rosdep update || true

# ------------------------------------------------------------
# STEP 8: Python libraries commonly used in robotics
# ------------------------------------------------------------
# RUN python3 -m pip install --upgrade numpy scipy matplotlib opencv-python pyyaml

# ------------------------------------------------------------
# STEP 9: Create and configure catkin workspace
# ------------------------------------------------------------
ARG WORKSPACE=/workspace/catkin_ws
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

# Optional: clone repo during build (pass REPO_URL as build arg)
ARG REPO_URL
ARG BRANCH=main
RUN if [ -n "${REPO_URL}" ]; then \
      git clone --depth 1 --branch ${BRANCH} "${REPO_URL}" src/polaris_gem_e2 || \
      echo "⚠️ Clone failed (maybe private repo, skipping)"; \
    fi

# ------------------------------------------------------------
# STEP 10: Install ROS package dependencies
# ------------------------------------------------------------
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true"

# ------------------------------------------------------------
# STEP 11: Build the workspace
# ------------------------------------------------------------
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ -f src/CMakeLists.txt ]; then \
        catkin_make -C .; \
    else \
        catkin_make; \
    fi || true"

# ------------------------------------------------------------
# STEP 12: Add non-root user for development
# ------------------------------------------------------------
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} || true \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} || true \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ------------------------------------------------------------
# STEP 13: Add entrypoint
# ------------------------------------------------------------
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENV PATH=$PATH:/opt/ros/${ROS_DISTRO}/bin

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
