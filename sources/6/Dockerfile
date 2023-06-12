FROM put/ai-rob-1:base

# https://github.com/ethz-asl/kalibr/wiki/installation

RUN apt install -y python3-catkin-tools python3-osrf-pycommon

RUN apt install -y \
    git wget autoconf automake nano \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    doxygen libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev

RUN apt install -y python3-dev python3-pip python3-scipy \
    python3-matplotlib ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx

WORKDIR /catkin_ws

RUN cd src && git clone https://github.com/ethz-asl/kalibr

RUN rm -rf build devel

RUN catkin init && catkin config --extend /opt/ros/noetic/ --merge-devel -DCMAKE_BUILD_TYPE=Release -j $(nproc) && catkin build

WORKDIR /

# Use rosrun kalibr <command_you_want_to_run_here>
