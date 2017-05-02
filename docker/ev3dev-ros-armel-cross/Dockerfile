FROM ev3dev/debian-stretch-armel-cross
USER root
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-get -y update && \
    apt-get install -y --no-install-recommends \
    ros-robot-dev \
    ros-perception-dev \
    ros-base-python-dev \
    ros-base-lisp-dev \
    rosbash \
    python-rosdep \
    ros-core-rosbuild-dev \
    ros-core-python-dev \
    ros-core-lisp-dev \
    ros-perception-python-dev \
    ros-perception-lisp-dev \
    ros-robot-python-dev \
    liburdfdom-dev \
    locate \
    rsync && \
    rm -rf /var/lib/apt/lists/*
