FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# update the dependencies
RUN apt update
RUN apt upgrade -y
RUN apt install git -y

# install eigen for linear algebra operations
RUN apt install -y libeigen3-dev

# create a repository for a workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# clone and build lart_msgs
RUN echo "hello"
RUN git clone https://github.com/FSLART/lart_msgs.git -b dev
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 4 --packages-select lart_msgs"

# get lart_common and put in tmp
WORKDIR /tmp
RUN git clone https://github.com/FSLART/lart_common.git -b dev

# copy the package to the container workspace
COPY . /ros2_ws/src/t24e_ekf
# copy lart_common to the package's include
RUN cp -r /tmp/lart_common /ros2_ws/src/t24e_ekf/include/lart_common
WORKDIR /ros2_ws

# build the package
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --symlink-install --parallel-workers 4"

