# Image
FROM ros:humble-ros-base

# 1. Installing tools as root user
RUN apt-get update && apt-get install -y \
    tree \
    python3-venv \
    python3-pip \
    python3-catkin-pkg \
    python3-lark \
    python3-empy \
    python3-colcon-common-extensions \
    usbutils \
    stlink-tools \
    git \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. bash as shell
ENV SHELL /bin/bash

# 3. Setting 'ubuntu' user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG PROJECT_NAME

# 4. Because there is no 'ubuntu' user on Ubuntu 22.04
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG dialout,plugdev $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 5. The user is now ubuntu for special config
USER $USERNAME 
ENV HOME=/home/$USERNAME
WORKDIR $HOME

# 6. Installing missing packages as 'ubuntu' user
RUN pip3 install --upgrade --user platformio 

# 6.1 Creating the penv directory because platformio needs it to use catking_pkg for example
RUN python3 -m venv /home/$USERNAME/.platformio/penv

# 6.2 Upgrade pip inside the penv and install the missing packages
RUN /home/$USERNAME/.platformio/penv/bin/pip install --upgrade pip && \
    /home/$USERNAME/.platformio/penv/bin/pip install catkin_pkg lark empy==3.3.4

# 7. PATH and ROS config in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && \
    echo 'export LIBCLANG_PATH=/usr/lib/llvm-14/lib' >> ~/.bashrc \
    echo "alias l='ls -la'" >> ~/.bashrc && \
    echo "alias me='echo ${PROJECT_NAME}'" >> ~/.bashrc
