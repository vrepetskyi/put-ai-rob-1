FROM put/ai-rob-1:base

RUN apt install -y ros-noetic-cv-camera ros-noetic-usb-cam

RUN cd /catkin_ws/src && git clone https://github.com/dominikbelter/sensors_demos_gazebo

RUN cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
