FROM put/ai-rob-1:base

WORKDIR /catkin_ws/src

RUN git clone https://github.com/mbed92/robotics-ai-classes.git

RUN git clone -b calibration_devel https://github.com/fmauch/universal_robot

RUN rosdep update

RUN rosdep install --from-paths . --ignore-src -r -y

RUN pip install pybotics

# A quick fix
RUN cd universal_robot/ur_description/launch && ln -s load_ur3.launch ur3_upload.launch

WORKDIR /

RUN cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
