FROM osrf/ros:noetic-desktop-full 

# Update the mirrors and upgrade the packages
RUN apt update -y && \
    apt upgrade -y
    
# Install missing tools
RUN apt install -y \
        neovim \
        wget \
        git \
        pip \
        evince

# Make python use python3
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Source ros and catkin on shell startup
RUN cd /root && \
    touch .bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> .bashrc && \
    echo 'source /catkin_ws/devel/setup.bash' >> .bashrc

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Init catkin workspace
RUN source /opt/ros/noetic/setup.bash && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws && \
    catkin_make
