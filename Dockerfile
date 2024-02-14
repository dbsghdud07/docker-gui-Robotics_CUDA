# 기반 이미지를 선택합니다.
FROM nvcr.io/nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# 환경 변수를 설정합니다.
ENV ROS_DISTRO=foxy

RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

RUN apt-get install --no-install-recommends -yqqq \
    python3-pip

COPY install_dev_tools.sh /tmp/install_dev_tools.sh
RUN chmod +x /tmp/install_dev_tools.sh

# 필요한 패키지를 설치합니다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=34" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /root/.bashrc && \
    echo "echo 'ROS2 environment set, Domain = 34'" >> /root/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source ~/catkin_ws/install/local_setup.bash" >> /root/.bashrc && \
    echo "alias cw='cd ~/catkin_ws'" >> /root/.bashrc && \
    echo "alias cs='cd ~/catkin_ws/src'" >> /root/.bashrc && \
    echo "alias cb='cd ~/catkin_ws && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbp='cd ~/catkin_ws && colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> /root/.bashrc && \
    echo "alias gb='gedit ~/.bashrc'" >> /root/.bashrc && \
    echo "alias rt='ros2 topic list'" >> /root/.bashrc && \
    echo "alias re='ros2 topic echo'" >> /root/.bashrc && \
    echo "alias rn='ros2 node list'" >> /root/.bashrc && \
    echo "alias testpub='ros2 run demo_nodes_cpp talker'" >> /root/.bashrc && \
    echo "alias testsub='ros2 run demo_nodes_cpp listener'" >> /root/.bashrc

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# setup sources.list
RUN echo "deb http://snapshots.ros.org/foxy/final/ubuntu focal main" > /etc/apt/sources.list.d/ros2-snapshots.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO foxy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-core=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod 777 /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro foxy

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*
