FROM vrepetskyi/rob:lab2

RUN apt-get install -y ros-noetic-turtlesim ros-noetic-rqt-graph wget

WORKDIR /catkin_ws

RUN cd src && git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git

RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN wget https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/tu-darmstadt-ros-pkg/Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag

WORKDIR /
