FROM randaz81/r1slam:cuda10_0
LABEL maintainer="marco.randazzo@iit.it"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_UPDATED_ON=20_1_2020

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
RUN mkdir isaac-navsim
RUN tar xf IsaacSimProject_1.2.1238533.gitdeps.tar.gz -C $robotology_install_folder/isaac/isaac-sim
RUN tar xf isaac-sdk-2019.2-30e21124.tar.xz           -C $robotology_install_folder/isaac/isaac-sdk
RUN tar xf isaac_navsim-2019-07-23.tar.xz             -C $robotology_install_folder/isaac/isaac-navsim
RUN rm IsaacSimProject_1.2.1238533.gitdeps.tar.gz isaac-sdk-2019.2-30e21124.tar.xz isaac_navsim-2019-07-23.tar.xz

RUN sudo apt-get install mesa-utils
RUN pip install tf-nightly
RUN python -c "import tensorflow"

WORKDIR  $robotology_install_folder/isaac/isaac-sdk
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=
ENV TZ=Europe/Rome
EXPOSE 3000-3010

##############################
# The following commands are not automatically performed by now and should be executed the first time to complete the built image
#
# This is the repo where the bridge stuff is located
# RUN git clone https://github.com/randaz81/isaac-bridge
#
# this installs the dependicies required to build isaac-sdk
# RUN $robotology_install_folder/isaac/isaac-sdk/engine/build/scripts/install_dependencies.sh
# 
# Bazel build
# WORKDIR  $robotology_install_folder/isaac/isaac-sdk
# RUN bazel build ...
#
# After build, save the docker image as a new container: eg. docker commit 8d8fcda30d24 randaz81/isaac:built-cuda10-0_v2
#
# The following are unecessary
# RUN bazel build //apps/tutorials/ping
# RUN bash -c "source /home/user1/.bashrc"
# RUN env
###############################

