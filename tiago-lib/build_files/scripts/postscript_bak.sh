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
  libcanberra-gtk-module

aptitude -y install \
  gcc-5-base=5.4.0-6ubuntu1~16.04.6 \
  libstdc++6=5.4.0-6ubuntu1~16.04.6 \
  libudev1=229-4ubuntu21 \
  libuuid1=2.27.1-6ubuntu3.3 \
  perl-base=5.22.1-9ubuntu0.2 \
  libc6=2.23-0ubuntu10

aptitude -y install \
  pal-erbium-pal-metapkg-development-tiago-37-gripper

rosdep init

mv /00sshd  /etc/apt/apt.conf.d/00sshd

apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AF84A6C9D128D11F

apt -y update
# echo "n" | apt -y upgrade

apt -y install \
  gdb \
  swig \
  doxygen \

  libudev-dev=229-4ubuntu21.15 \
  libkmod2=22-1ubuntu5.1 \
  kmod linux-sound-base alsa-base alsa-tools alsa-utils

  python3-pip libopenblas-dev python3-scipy cython libhdf5-dev python3-h5py portaudio19-dev

  python-pip \
  python-pyaudio \
  python-soundfile \

  libpulse-dev \
  libsigsegv-dev \
  ros-kinetic-rtabmap \
  ros-kinetic-rtabmap-ros \
  ros-kinetic-find-object-2d \
  python-wstool \
  python-catkin-pkg \
  python-catkin-tools \
  build-essential

. /opt/pal/erbium/setup.sh
NUM_CPUS=$(grep -c ^processor /proc/cpuinfo)

mkdir /code

# GeographicLib
curl -L https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.49.tar.gz/download | tar xz -C /code
cd /code/GeographicLib-1.49/ && mkdir build && cd build
../configure
make -j$NUM_CPUS
make install

# libQGLViewer
curl -L http://libqglviewer.com/src/libQGLViewer-2.6.3.tar.gz | tar xz -C /code
cd /code/libQGLViewer-2.6.3/QGLViewer
qmake
make -j$NUM_CPUS
make install

# gtsam
curl -L https://github.com/borglab/gtsam/archive/4.0.0-alpha2.tar.gz | tar xz -C /code
cd /code/gtsam-4.0.0-alpha2 && mkdir build && cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j$NUM_CPUS
make install

# g2o
git clone https://github.com/RainerKuemmerle/g2o.git /code/g2o
cd /code/g2o && mkdir build && cd build
cmake ..
make -j$NUM_CPUS
make install

# rtabmap
git clone https://github.com/introlab/rtabmap.git /code/rtabmap
cd /code/rtabmap/build/
cmake ..
make -j$NUM_CPUS
make install

# gnu M4
curl -L http://ftp.gnu.org/gnu/m4/m4-1.4.18.tar.gz | tar xz -C /code
cd /code/m4-1.4.18/ && mkdir build && cd build
../configure
make -j$NUM_CPUS
make install

# bison
curl -L http://ftp.gnu.org/gnu/bison/bison-3.3.2.tar.gz | tar xz -C /code
cd /code/bison-3.3.2/ && mkdir build && cd build
../configure
make -j$NUM_CPUS
make install

# sphinxbase
curl -L https://vorboss.dl.sourceforge.net/project/cmusphinx/sphinxbase/5prealpha/sphinxbase-5prealpha.tar.gz | tar xz -C /code
mv /code/sphinxbase-5prealpha /code/sphinxbase
cd /code/sphinxbase
./configure
make -j$NUM_CPUS
make install

# pocketsphinx
curl -L https://netcologne.dl.sourceforge.net/project/cmusphinx/pocketsphinx/5prealpha/pocketsphinx-5prealpha.tar.gz | tar xz -C /code
cd /code/pocketsphinx-5prealpha
./configure
make -j$NUM_CPUS
make install

pip install pocketsphinx

# vscode
curl -L https://go.microsoft.com/fwlink/?LinkID=760868 -o code.deb
apt -y install libxtst6 ./code.deb
rm -rf code.deb

# pycharm
curl -L https://download-cf.jetbrains.com/python/pycharm-community-2019.1.1.tar.gz | tar xz -C /opt

mkdir /tiago_src
mkdir -p /tiago_ws

