FROM osrf/ros:kinetic-desktop-full
MAINTAINER Nikolay Dema "ndema2301@gmail.com"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    apt-utils \
    gnupg2 \
    net-tools \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim \
    python-catkin-tools


# Environment configuration

ENV  DRIVERS_WS /drivers_ws
ENV  ROS_DISTRO kinetic
ENV  ROS_INSTALL_PATH "/opt/ros/$ROS_DISTRO/"


RUN echo "source ${ROS_INSTALL_PATH}/setup.bash" >> ~/.bashrc
RUN echo "source ${DRIVERS_WS}/devel/setup.bash" >> ~/.bashrc

RUN /bin/bash -c "source ~/.bashrc"


# Install deps and additional ROS packages

COPY ${DRIVERS_WS} ${DRIVERS_WS}

RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-kinetic-teleop-twist-keyboard && \
    rosdep update --include-eol-distros && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths ${DRIVERS_WS} --ignore-src --rosdistro=kinetic -y

RUN rm -R ${DRIVERS_WS}
