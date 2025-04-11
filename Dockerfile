ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rqt-image-view

ARG UID
ARG GID

RUN groupadd -g ${GID} user && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} user

RUN usermod -a -G video user

RUN echo "source /opt/ros/humble/setup.bash" >> /home/user/.bashrc

USER user
WORKDIR /home/user/
