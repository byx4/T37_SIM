FROM ubuntu:24.04
 
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy
ENV TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO}
 
# Use bash as default shell (so `source` works)
SHELL ["/bin/bash", "-lc"]
 
# Locales
RUN apt-get update && apt-get install -y --no-install-recommends locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && locale \
 && rm -rf /var/lib/apt/lists/*
 
# Universe repo
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common \
 && add-apt-repository -y universe \
 && rm -rf /var/lib/apt/lists/*
 
# curl + ROS apt source (latest via GitHub release)
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates \
 && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
 && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
 && dpkg -i /tmp/ros2-apt-source.deb \
 && rm -f /tmp/ros2-apt-source.deb \
 && rm -rf /var/lib/apt/lists/*
 
# Install ROS 2 Jazzy + tools
RUN apt-get update \
 && apt-get -y upgrade \
 && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      python3-rosdep python3-colcon-common-extensions python3-argcomplete \
      build-essential cmake git \
 && rm -rf /var/lib/apt/lists/*
 
# rosdep (safe in container)
RUN rosdep init || true && rosdep update
 
# QoL: argcomplete + auto-source ROS in all interactive shells
RUN activate-global-python-argcomplete || true \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
 
WORKDIR /root/ws

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"] 
CMD ["/bin/bash"]