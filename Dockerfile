FROM ros:melodic-ros-base-bionic 
ARG DEBIAN_FRONTEND=noninteractive
RUN \
	apt update -y && \
	apt upgrade -y && \
	apt install -y libpcl-dev && \
	apt install -y ros-melodic-pcl-ros && \
	apt install -y pcl-tools && \
	apt install -y ros-melodic-cv-bridge && \
	apt install -y libyaml-cpp-dev && \
	apt install -y libpcap-dev



RUN mkdir /workspace
	
WORKDIR /workspace

	
