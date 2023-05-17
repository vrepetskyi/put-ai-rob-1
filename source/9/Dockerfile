FROM put/ai-rob-1:base

RUN git clone -b calibration_devel https://github.com/fmauch/universal_robot /catkin_ws/src/universal_robot

RUN cd /catkin_ws/src && git clone https://github.com/mbed92/robotics-ai-classes.git /catkin_ws/src/robotics-ai-classes

RUN rosdep update && rosdep install --from-paths /catkin_ws/src --ignore-src -y

RUN cd /catkin_ws/src/universal_robot/ur_description/launch && ln -s load_ur3.launch ur3_upload.launch
