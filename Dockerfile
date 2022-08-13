FROM ros:melodic-ros-base-bionic 
ARG DEBIAN_FRONTEND=noninteractive
RUN \
	apt update -y && \
	apt upgrade -y && \
	apt install -y vim

RUN \
	apt install -y libpcl-dev && \
	apt install -y ros-melodic-pcl-ros && \
	apt install -y pcl-tools && \
	apt install -y ros-melodic-cv-bridge && \
	apt install -y libyaml-cpp-dev && \
	apt install -y libpcap-dev

RUN \
	apt install -y python3-pip && \
	apt install -y python3-catkin-tools python3-vcstool python3-osrf-pycommon

RUN pip3 install rospkg
RUN pip3 install numpy
RUN pip3 install scipy

RUN apt install -y software-properties-common
RUN add-apt-repository ppa:sweptlaser/python3-pcl
RUN apt update
RUN apt install -y python3-pcl

RUN apt install -y ros-melodic-rosbridge-server
RUN mkdir /workspace
	
WORKDIR /workspace

	
