FROM ubuntu:18.04

RUN \
    apt update && \
    apt install -y gnupg && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

RUN \
    echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main' > /etc/apt/sources.list.d/gazebo-stable.list

RUN \
    apt update && \
    apt install -y debhelper cmake git python-future libgazebo9-dev libns3-dev libgsl-dev

WORKDIR /gzuav_build
