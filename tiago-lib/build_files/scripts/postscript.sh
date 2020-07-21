#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
echo 'export LANG=C.UTF-8' >> $SINGULARITY_ENVIRONMENT
echo 'export LC_ALL=C.UTF-8' >> $SINGULARITY_ENVIRONMENT
. $SINGULARITY_ENVIRONMENT

echo "deb [trusted=yes] file:/pkgs ./" > /etc/apt/sources.list

apt update
apt install -y \
  aptitude \
  apt-transport-https \
  ca-certificates \
  mesa-utils \
  libatk-adaptor \
  libgail-common \
  libcanberra-gtk-module \
  libcanberra-gtk3-module

aptitude -y remove \
  libc6:i386 \
  libgcc1:i386 \
  libx11-6:i386 \
  libx11-dev:i386 \
  libxau-dev:i386 \
  libxau6:i386 \
  libxcb1:i386 \
  libxcb1-dev:i386 \
  libxdmcp-dev:i386 \
  libxdmcp6:i386 \
  libxext-dev:i386 \
  libxext6:i386

aptitude -y install \
  gcc-5-base=5.4.0-6ubuntu1~16.04.6 \
  libstdc++6=5.4.0-6ubuntu1~16.04.6 \
  libudev1=229-4ubuntu21 \
  libuuid1=2.27.1-6ubuntu3.3 \
  perl-base=5.22.1-9ubuntu0.2 \
  libperl5.22=5.22.1-9ubuntu0.2 \
  perl=5.22.1-9ubuntu0.2 \
  libc6=2.23-0ubuntu10 \
  libglib2.0-0=2.48.2-0ubuntu1 \
  binutils=2.26.1-1ubuntu1~16.04.6

aptitude -y install \
  pal-erbium-pal-metapkg-development-tiago-37-gripper

rosdep init

mv /00sshd  /etc/apt/apt.conf.d/00sshd

apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AF84A6C9D128D11F

apt -y update
# echo "n" | apt -y upgrade

apt -y install \
  gdb \
  doxygen \
  libxtst6 \
  flex \
  libxml++2.6-dev \
  ros-kinetic-rtabmap \
  ros-kinetic-rtabmap-ros \
  ros-kinetic-find-object-2d \
  ros-kinetic-aruco-ros \
  ros-kinetic-aruco-detect \
  pal-erbium-aruco-ros \
  python-wstool \
  python-catkin-pkg \
  python-catkin-tools \
  build-essential

# only installed for the dependecies
# apt -y remove \
#   ros-kinetic-rtabmap \
#   ros-kinetic-rtabmap-ros

# pyaudio deps
apt -y install \
  libjack-dev \
  libjack0 \
  libportaudio2 \
  libportaudiocpp0 \
  portaudio19-dev

# pip and pyaudio etc
curl "https://bootstrap.pypa.io/get-pip.py" -O
python get-pip.py
pip install soundfile pyaudio python-statemachine

# . /opt/pal/erbium/setup.sh
NUM_CPUS=$(grep -c ^processor /proc/cpuinfo)
mkdir /code

git clone https://github.com/iocchi/PetriNetPlans.git /code/PetriNetPlans
cd /code/PetriNetPlans/PNP && mkdir build && cd build
cmake ..
make -j$NUM_CPUS
make install

cd /code/PetriNetPlans/PNPgen && mkdir build && cd build
cmake ..
make -j$NUM_CPUS
make install



# curl -L https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.49.tar.gz/download | tar xz -C /code
# cd /code/GeographicLib-1.49/ && mkdir build && cd build
# ../configure
# make -j$NUM_CPUS
# make install

# curl -L http://libqglviewer.com/src/libQGLViewer-2.6.3.tar.gz | tar xz -C /code
# cd /code/libQGLViewer-2.6.3/QGLViewer
# qmake
# make -j$NUM_CPUS
# make install


# curl -L https://github.com/borglab/gtsam/archive/4.0.0-alpha2.tar.gz | tar xz -C /code
# cd /code/gtsam-4.0.0-alpha2 && mkdir build && cd build
# cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
# make -j$NUM_CPUS
# make install


# git clone https://github.com/RainerKuemmerle/g2o.git /code/g2o
# cd /code/g2o && mkdir build && cd build
# cmake ..
# make -j$NUM_CPUS
# make install


# git clone https://github.com/introlab/rtabmap.git /code/rtabmap
# cd /code/rtabmap/build/
# cmake ..
# make -j$NUM_CPUS
# make install

# curl -L https://go.microsoft.com/fwlink/?LinkID=760868 -o code.deb
# apt -y install libxtst6 ./code.deb
# rm -rf code.deb

curl -L https://download-cf.jetbrains.com/python/pycharm-community-2019.1.2.tar.gz | tar xz -C /opt

mkdir /tiago_src
mkdir -p /tiago_ws
