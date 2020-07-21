#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
if [ -f "/.singularity.d/env/91-environment.sh" ]; then
  . /.singularity.d/env/91-environment.sh
fi

apt -y install \
  gdb \
  doxygen \
  flex \
  flac \
  bison \
  libxml++2.6-dev \
  unzip \
  mongodb \
  mercurial \
  default-jdk \
  software-properties-common \
  ros-kinetic-mongodb-store \
  ros-kinetic-rtabmap \
  ros-kinetic-rtabmap-ros \
  ros-kinetic-find-object-2d \
  ros-kinetic-aruco-ros \
  ros-kinetic-aruco-detect \
  ros-kinetic-timed-roslaunch \
  ros-kinetic-trac-ik-kinematics-plugin \
  pal-erbium-aruco-ros \
  python-wstool \
  python-catkin-pkg \
  python-catkin-tools \
  build-essential \
  clang-format-6.0 \
  libqt4-opengl \
  python-qt4-gl
  # ros-kinetic-moveit \
  # ros-kinetic-moveit-visual-tools \

# only installed for the dependecies
# apt -y remove \
#   ros-kinetic-rtabmap \
#   ros-kinetic-rtabmap-ros

# python 3.7
# add-apt-repository -y ppa:deadsnakes/ppa
# apt -y update
# apt -y install \
#   python3.7 \
#   python3.7-dev

#  pyaudio deps
apt -y install \
  libaudio2 \
  libportaudio2 \
  libportaudiocpp0 \
  portaudio19-dev \
  pulseaudio \
  pulseaudio-module-bluetooth \
  pulseaudio-utils \
  alsa-base \
  alsa-firmware-loaders \
  alsa-oss \
  alsa-tools \
  alsa-utils

# pip and pyaudio etc
if [ ! -f get-pip.py ]; then
  curl "https://bootstrap.pypa.io/get-pip.py" -O
fi
# if ! which pip3 > /dev/null; then
#   python3.7 get-pip.py
# fi
if ! which pip2 > /dev/null; then
  python2 get-pip.py
fi

pip2 install virtualenv
# pip3 install soundfile pyaudio rospkg catkin_pkg

# virtualenv setup
if [ ! -f "$VENV/bin/python" ]; then
  virtualenv $VENV
fi

source $VENV/bin/activate
pip install \
  rospkg \
  soundfile \
  pyaudio \
  python-statemachine \
  dialogflow \
  PyYAML \
  empy \
  netifaces \
  defusedxml \
  numpy \
  requests
  # tensorflow \
  # keras pillow matplotlib scikit-learn scikit-image pydot GraphViz PyGLM==0.4.8b1

# copy sip from /usr/lib/python to python venv
if [ ! -f "/code/python-env/lib/python2.7/sip.x86_64-linux-gnu.so" ]; then
  cp /usr/lib/python2.7/dist-packages/sip* /code/python-env/lib/python2.7/
fi

# copy PySide2 from /usr/lib/python to python venv
if [ ! -d "/code/python-env/lib/python2.7/PySide2" ]; then
  cp -r /usr/lib/python2.7/dist-packages/PySide2 /code/python-env/lib/python2.7/
fi

# copy OpenGL from /usr/lib/python to python venv
if [ ! -d "/code/python-env/lib/python2.7/OpenGL" ]; then
  cp -r /usr/lib/python2.7/dist-packages/OpenGL /code/python-env/lib/python2.7/
fi

# copy PyQt4, PyQt5 from /usr/lib/python to python venv
if [ ! -d "/code/python-env/lib/python2.7/PyQt4" ]; then
  cp -r /usr/lib/python2.7/dist-packages/PyQt* /code/python-env/lib/python2.7/
fi

# install PNP
if [ ! -d "/code/PetriNetPlans" ]; then
  git clone https://github.com/iocchi/PetriNetPlans.git /code/PetriNetPlans
  cd /code/PetriNetPlans/PNP && mkdir build && cd build
  cmake ..
  make -j$NUM_CPUS
  make install

  cd /code/PetriNetPlans/PNPgen && mkdir build && cd build
  cmake ..
  make -j$NUM_CPUS
  make install
fi

# fetch gazebo models
DEST=/usr/share/gazebo-7/models
if [ ! -d "$DEST/cafe" ]; then
  ZIP=models.zip

  rm -rf $DEST/*
  curl -o $ZIP "https://bitbucket.org/osrf/gazebo_models/get/e6d645674e8a.zip"
  unzip -n -q -d "$DEST" "$ZIP" && f=("$DEST"/*) && mv "$DEST"/*/* "$DEST" && rm -rf "${f[@]}"
  rm -f $ZIP
fi

# Picovoice code
if [ ! -d "/code/Picovoice/porcupine" ]; then
  git clone https://github.com/Picovoice/Porcupine.git /code/Picovoice/porcupine
fi
if [ ! -d "/code/Picovoice/cheetah" ]; then
  git clone https://github.com/Picovoice/cheetah.git /code/Picovoice/cheetah
fi

# . /opt/pal/erbium/setup.sh

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
