FROM put/ai-rob-1:base

WORKDIR /catkin_ws/src

RUN git clone https://bitbucket.org/AndyZe/pid.git

WORKDIR /

RUN cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
