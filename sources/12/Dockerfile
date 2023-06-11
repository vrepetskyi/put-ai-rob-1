FROM put/ai-rob-1:base

WORKDIR /catkin_ws/src

RUN git clone https://github.com/MichalNowicki/KalmanFilter.git

RUN rosdep update

RUN rosdep install --from-paths . --ignore-src -r -y

WORKDIR /

RUN cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
