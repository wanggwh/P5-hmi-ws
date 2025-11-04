FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Setup locales
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    git wget curl gnupg2 lsb-release \
    build-essential cmake udev sudo \
    software-properties-common python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# System dependencies for Kivy/KivyMD
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    gstreamer1.0-tools python3-dev

# Install other useful tools
RUN apt-get update && apt-get install -y \
    nano \
    python3-vcstool \
    python3-colcon-common-extensions \
    x11-apps \
    vim \
    iputils-ping \
    python3-empy \
    python3-numpy \
    python3-lark \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-venv python3-pip

COPY . /home/P5-hmi-ws/
WORKDIR /home/P5-hmi-ws/

# Create virtual environment and ensure pip is present
RUN python3 -m venv kivy_venv && \
    kivy_venv/bin/python -m ensurepip --upgrade && \
    kivy_venv/bin/python -m pip install --upgrade pip setuptools wheel --break-system-packages

# Install Python dependencies in the venv
RUN kivy_venv/bin/python -m pip install numpy lark empy pyyaml catkin_pkg "kivy[base]" kivy_examples kivymd --break-system-packages

# Add venv to PATH so it's used by default in the container
ENV PATH="kivy_venv/bin:$PATH"

# Automatically source ROS and workspace setups in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/P5-hmi-ws/install/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]
ENV ROS_DOMAIN_ID=69