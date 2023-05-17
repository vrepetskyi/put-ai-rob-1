FROM put/rob1:base

WORKDIR /catkin_ws/src

RUN git clone https://github.com/mbed92/robotics-ai-classes.git

RUN git clone -b calibration_devel https://github.com/fmauch/universal_robot

WORKDIR /catkin_ws

RUN rosdep update

RUN rosdep install --from-paths src --ignore-src -y

RUN source /opt/ros/noetic/setup.bash && catkin_make

WORKDIR /
