FROM put/ai-rob-1:base

RUN cd /catkin_ws/src && git clone https://github.com/MichalNowicki/KalmanFilter.git

RUN rosdep update

RUN rosdep install --from-paths /catkin_ws/src --ignore-src -r -y

RUN cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
