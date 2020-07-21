#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
echo 'export LANG=C.UTF-8' >> $SINGULARITY_ENVIRONMENT
echo 'export LC_ALL=C.UTF-8' >> $SINGULARITY_ENVIRONMENT
echo 'export NUM_CPUS=$(grep -c ^processor /proc/cpuinfo)'  >> $SINGULARITY_ENVIRONMENT
echo 'export VENV=/code/python-env'  >> $SINGULARITY_ENVIRONMENT
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
  binutils=2.26.1-1ubuntu1~16.04.6 \
  libbz2-1.0=1.0.6-8

aptitude -y install \
  pal-erbium-pal-metapkg-development-tiago-37-gripper

rosdep init

mv /00sshd  /etc/apt/apt.conf.d/00sshd

apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AF84A6C9D128D11F

# upgrade packages
apt-mark hold tzdata
apt -y update
# echo "n" | apt -y upgrade

# file watcher limits
echo fs.inotify.max_user_watches=524288 | tee -a /etc/sysctl.conf && sysctl -p

# remove pkgs and repo
rm -rf /pkgs/*
rm -rf /etc/apt/sources.list
mv /sources.list /etc/apt/sources.list
apt -y update

# install vscode
curl -L https://go.microsoft.com/fwlink/?LinkID=760868 -o code.deb
chown _apt:root ./code.deb
apt -y install ./code.deb
rm code.deb

# install pycharm
curl -L https://download-cf.jetbrains.com/python/pycharm-community-2019.2.1.tar.gz | tar xz -C /opt

mkdir -p /tiago_ws
mkdir -p /code

# install local software
/scripts/postscript_current.sh

# make sure not left out of date in the container
rm -rf /scripts/*