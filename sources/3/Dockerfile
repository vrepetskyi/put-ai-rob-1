FROM put/ai-rob-1:base

RUN apt install -y ros-noetic-turtlesim ros-noetic-rqt-graph

RUN cd /catkin_ws/src && git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git

RUN cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
