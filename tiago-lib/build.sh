#!/bin/bash

if [ -z $1 ]; then
  echo "please add --{nv, hybrid, nonnv} to command"
  exit 1
fi

if [ ! -d "$(pwd)/build_files/pkgs" ]; then
  if [ ! -d "$HOME/OneDrive/dev/tiago/pkgs" ]; then
    echo "No OneDrive pkgs to link to, please make sure ~/OneDrive/dev/tiago/pkgs exists, or edit the ./build.sh file"
    exit 2
  else
    ln -s $HOME/OneDrive/dev/tiago/pkgs ./build_files/
  fi
fi

if [ "$@" = "--nv" ]; then
  echo "nv"
  sudo singularity build --sandbox tiago/ build_files/Singularity.nv
elif [ "$@" = "--hybrid" ]; then
  echo "hybrid"
  sudo singularity build --sandbox tiago/ build_files/Singularity.hybrid
elif [ "$@" = "--nonnv" ]; then
  echo "nonnv"
  sudo singularity build --sandbox tiago/ build_files/Singularity.nonnv
else
  echo "invalid: --{nv, hybrid, nonnv}"
  exit 1
fi

sudo singularity capability add --user ${USER} CAP_NET_RAW