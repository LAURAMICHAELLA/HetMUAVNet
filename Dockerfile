FROM ubuntu:18.04

LABEL Description="Docker image for MULTI-UAV NS-3 Network Simulator+Gazebo+ROS"

MAINTAINER Laura Ribeiro <laura.michaella@gmail.com>

RUN \
    apt update && \
    apt install -y \
	gnupg \
	git \
	mercurial \
  	wget \
  	vim \
  	autoconf \
  	bzr \
  	cvs \
  	unrar \
  	build-essential \
  	clang \
  	valgrind \
  	gsl-bin \
  	libgsl23 \
  	libgsl-dev \
  	flex \
  	bison \
  	libfl-dev \
  	tcpdump \
  	sqlite \
  	sqlite3 \
  	libsqlite3-dev \
  	libxml2 \
  	libxml2-dev \
  	vtun \
  	lxc \
	&& rm -rf /var/lib/apt/lists/

RUN \
    apt update && \
    apt install -y gnupg && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

RUN \
    echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main' > /etc/apt/sources.list.d/gazebo-stable.list

RUN \
    apt update && \
    apt install -y debhelper cmake git python-pip libgazebo9-dev libns3-dev libgsl-dev curl libgl

# QT4 components
RUN apt-get install -y \
  qtbase5-dev

# Python components
RUN apt-get install -y \
  python \
  python-dev \
  python-setuptools \
  cmake \
  libc6-dev \
  libc6-dev-i386 \
  g++-multilib \
  pip 

RUN apt update && \
    pip install dronekit

# Cloning & installing NS3-all-in-one
RUN hg clone http://code.nsnam.org/ns-3-allinone
RUN cd ns-3-allinone && python download.py -n ns-3.29 && python build.py --enable-examples --enable-tests
RUN cd ns-3-allinone/ns-3.29 && python test.py

WORKDIR /HetMUAVNet






