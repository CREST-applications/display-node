ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update && apt-get install -y\
    libopencv-dev \
    ros-${ROS_DISTRO}-rqt-image-view

RUN pip install \
    opencv-python \
    cv_bridge

ARG UID
ARG GID

RUN groupadd -g ${GID} user && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} user

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/user/.bashrc
RUN usermod -a -G video user

USER user
WORKDIR /home/user/
