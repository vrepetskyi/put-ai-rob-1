FROM put/ai-rob-1:base

WORKDIR /catkin_ws/src

RUN git clone https://github.com/MichalNowicki/KalmanFilter.git

WORKDIR /catkin_ws

RUN rosdep update

RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/noetic/setup.bash && catkin_make

WORKDIR /
