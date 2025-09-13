# syntax=docker/dockerfile:1.6
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Base system and locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales tzdata ca-certificates curl wget gnupg lsb-release software-properties-common \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && dpkg-reconfigure -f noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ROS 2 apt repository
RUN set -e; \
    apt-get update && apt-get install -y --no-install-recommends \
      gnupg2 curl \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 Humble + common dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-desktop \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      build-essential \
      cmake \
      git \
      nano \
      gedit \
      terminator \
      bash-completion \
      iputils-ping \
      net-tools \
      wget \
      sudo \
      mesa-utils \
      libgl1 \
      dbus-x11 \
    && rosdep init || true \
    && rm -rf /var/lib/apt/lists/*

# Non-root user with sudo
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME} \
    && usermod -aG sudo ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/99_${USERNAME} \
    && chmod 0440 /etc/sudoers.d/99_${USERNAME}

# Source ROS for all shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Workspace
RUN mkdir -p /workspace && chown ${UID}:${GID} /workspace
WORKDIR /workspace

# Entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]


