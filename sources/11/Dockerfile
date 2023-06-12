FROM put/ai-rob-1:base

RUN cd /catkin_ws/src && git clone https://bitbucket.org/AndyZe/pid.git

RUN cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
