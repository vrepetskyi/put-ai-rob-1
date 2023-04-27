FROM osrf/ros:noetic-desktop-full 

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get install -y ros-noetic-cv-camera ros-noetic-usb-cam ros-noetic-catkin git neovim

WORKDIR /catkin_ws

RUN mkdir src && cd src && git clone https://github.com/dominikbelter/sensors_demos_gazebo

RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN touch /root/.bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc

WORKDIR /
