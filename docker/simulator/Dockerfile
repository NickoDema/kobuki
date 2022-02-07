ARG from

FROM ${from}
MAINTAINER Nikolay Dema "ndema2301@gmail.com"


RUN apt-get update && apt-get install -y \
    apt-utils \
    mesa-utils \
    gnupg2 \
    net-tools \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim



# Environment configuration

ENV OS_DISTRO xenial
ENV ROS_DISTRO kinetic
ENV ROS_INSTALL_PATH "/opt/ros/$ROS_DISTRO/"
ENV SIM_WS /simulator_ws



# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone



# Install ROS desktop and Gazebo

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $OS_DISTRO main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y ros-kinetic-desktop gazebo7 && \
    echo "source $ROS_INSTALL_PATH/setup.bash"  >> ~/.bashrc && \
    echo "source ${SIM_WS}/devel/setup.bash" >> ~/.bashrc && \
    rosdep init && rosdep update



# Install catkin tools and additional ROS packages

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y python-catkin-tools \
                                                      ros-kinetic-teleop-twist-keyboard


# User WS dependings

COPY ${SIM_WS} ${SIM_WS}
RUN rosdep update --include-eol-distros && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths ${SIM_WS} --ignore-src --rosdistro=${ROS_DISTRO} -y

RUN rm -R ${SIM_WS}
