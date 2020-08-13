FROM randaz81/r1slam:cuda10.0_ubuntu18.04
LABEL maintainer="marco.randazzo@iit.it"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_UPDATED_ON=7_8_2020

# Update apt database
RUN sudo apt update

# set install_folder
USER user1
ENV robotology_install_folder /home/user1
ENV user1_home /home/user1
ENV robotology_install_user user1

# Install essentials
RUN mkdir $robotology_install_folder/isaac
COPY *.tar*  $robotology_install_folder/isaac/

WORKDIR $robotology_install_folder/isaac
RUN mkdir isaac-sdk
RUN mkdir isaac-sim
RUN tar xf isaac_sim_unity3d-20200527-a8205d23.tar.xz -C $robotology_install_folder/isaac/isaac-sim
RUN tar xf isaac-sdk-20200527-0159e2bab.tar.xz         -C $robotology_install_folder/isaac/isaac-sdk
RUN rm isaac-sdk-20200527-0159e2bab.tar.xz isaac_sim_unity3d-20200527-a8205d23.tar.xz

RUN sudo apt-get install -y mesa-utils python python-dev python-pip ipython python-setuptools python3-setuptools python3-pip
#RUN pip install --upgrade pip
#RUN pip install tf-nightly
#RUN pip install tensorflow==1.15
#RUN pip install tensorflow-gpu==1.15
#RUN python -c "import tensorflow"

WORKDIR  $robotology_install_folder/isaac/isaac-sdk
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=
ENV TZ=Europe/Rome
EXPOSE 3000-3010

# Prepare Unity installation
COPY UnityHub.AppImage $robotology_install_folder/isaac/

# This seems to work, with some red warnings
RUN $robotology_install_folder/isaac/isaac-sdk/engine/build/scripts/install_dependencies.sh

##############################
# The following commands are not automatically performed by now and should be executed the first time to complete the built image
#
# This is the repo where the bridge stuff is located
# RUN git clone --branch isaac2020.1 https://github.com/robotology/isaac-bridge
#
# Links
# RUN ln -s ~/isaac/isaac-sim  ~/isaac_sim_unity3d
# RUN ln -s /usr/local /home/user1/yarp-install
#
# this installs the dependencies required to build isaac-sdk
# RUN $robotology_install_folder/isaac/isaac-sdk/engine/build/scripts/install_dependencies.sh
# 
# Bazel build
# WORKDIR  $robotology_install_folder/isaac/isaac-sdk
# RUN bazel build ...
# 
# add the following to bashrc
# source /usr/local/lib/bazel/bin/bazel-complete.bash
#
# Install Unity editor (docker requires --privileged)
# ./UnityHub.AppImage unityhub://2018.3.11f1/5063218e4ab8
#
# After build, save the docker image as a new container, eg:
# docker ps
# docker commit 8d8fcda30d24 randaz81/isaac:built-2019_3
#
# The following are unecessary
# RUN bazel build //apps/tutorials/ping
# RUN bash -c "source /home/user1/.bashrc"
# RUN env
###############################

