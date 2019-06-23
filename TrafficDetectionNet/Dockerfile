# Udacity capstone project dockerfile
#FROM tensorflow/tensorflow:latest-gpu
FROM tensorflow/tensorflow:1.3.0-gpu

RUN apt-get clean

# Install tools
RUN apt-get update
RUN apt-get install vim git tree -y

# install python dependencies
RUN pip install pillow lxml matplotlib cython
RUN apt-get install python-tk python-yaml -y

# install protobuf-compiler
RUN mkdir prot
RUN cd prot
RUN curl -OL https://github.com/google/protobuf/releases/download/v3.2.0/protoc-3.2.0-linux-x86_64.zip
RUN unzip protoc-3.2.0-linux-x86_64.zip -d protoc3
RUN mv protoc3/bin/* /usr/local/bin/
RUN mv protoc3/include/* /usr/local/include/
RUN ln -s /usr/local/bin/protoc /usr/bin/protoc
RUN cd ..
RUN rm prot -rf

RUN export PYTHONPATH=$PYTHONPATH:/capstone/models/research/object_detection:/capstone/models/research:/capstone/models/research/slim

# install ros
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#RUN apt-get update
#RUN apt-get install ros-kinetic-desktop -y
#RUN rosdep init
#RUN rosdep update
#RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
#RUN source ~/.bashrc

# Install Dataspeed DBW https://goo.gl/KFSYi1 from binary
# adding Dataspeed server to apt
#RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
#RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
#RUN apt-get update

# setup rosdep
#RUN sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
#RUN rosdep update
#RUN apt-get install -y ros-$ROS_DISTRO-dbw-mkz
#RUN apt-get upgrade -y
# end installing Dataspeed DBW

# install python packages
#RUN apt-get install -y python-pip
#COPY requirements.txt ./requirements.txt
#RUN pip install -r requirements.txt

# install required ros dependencies
#RUN apt-get install -y ros-$ROS_DISTRO-cv-bridge
#RUN apt-get install -y ros-$ROS_DISTROpcl-ros
#RUN apt-get install -y ros-$ROS_DISTROimage-proc

# socket io
#RUN apt-get install -y netbase

# prepare environment
RUN mkdir /capstone
VOLUME ["/capstone"]
WORKDIR /capstone
