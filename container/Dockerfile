# Use ROS 2 Jazzy as the base image
FROM ros:jazzy
ARG USERNAME=zsl
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Remove any existing default user
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel `id -un $USER_UID`; fi

# Create a new user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install common tools and ROS 2 dependencies
RUN apt-get update && \
    apt-get install -y python3-pip python3-pandas python3-matplotlib libopencv-dev curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros2.key && \
    gpg --dearmor -o /usr/share/keyrings/ros2-archive-keyring.gpg /tmp/ros2.key && \
    echo 'deb [signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main' | tee /etc/apt/sources.list.d/ros2-latest.list && \    
    apt-get update && \
    apt-get install -y ros-jazzy-ros-base && \
    rm -rf /var/lib/apt/lists/*

# Set default shell to Bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc
ENV SHELL /bin/bash

# Switch to the regular user
USER $USERNAME
CMD ["/bin/bash"]