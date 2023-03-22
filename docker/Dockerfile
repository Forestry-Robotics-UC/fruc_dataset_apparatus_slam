FROM ros:melodic-perception

ENV ROS_ROOT=/opt/ros/melodic
#ENV ROS_PYTHON_VERSION=3
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash","-c"]

# Install packages
RUN apt-get update \
    && apt-get install -y \
    # Basic utilities
    build-essential \
    apt-utils \
    curl \
    git \
    wget \
    vim \
    nano 

# Install some python packages
RUN apt-get -y install \
        python \
        python-pip \
        python-serial \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        python-setuptools \
        python-rosdep && \
        pip install pybind11 \
        catkin_tools

#Install OpenCV
WORKDIR /root
RUN git clone https://github.com/opencv/opencv.git
WORKDIR /root/opencv
RUN git checkout tags/3.4.0
RUN mkdir build
WORKDIR /root/opencv/build
RUN cmake \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DWITH_CUDA=OFF \
        -DBUILD_DOCS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        ..
RUN make install
ENV OpenCV_DIR=/root/opencv

# Add ROS packages
RUN apt-get install -y ros-melodic-rviz

# Clean-up
RUN apt-get clean

WORKDIR /root/
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
WORKDIR /root/opencv/build
RUN cmake ..
RUN make
RUN sudo make install

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS
RUN catkin init
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git src/
RUN git clone https://github.com/Livox-SDK/livox_mapping.git src/livox_mapping

RUN source /opt/ros/melodic/setup.bash && catkin build
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

#ENTRYPOINT ["/usr/local/bin/catkin_entrypoint.sh"]
CMD ["bash"]