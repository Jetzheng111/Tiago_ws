#!/bin/bash

if [ ! -d "$(pwd)/build_files/pkgs" ]; then
  if [ ! -d "$HOME/OneDrive/dev/tiago/pkgs" ]; then
    echo "No OneDrive pkgs to link to, please make sure ~/OneDrive/dev/tiago/pkgs exists, or edit the ./build.sh file"
    exit 2
  else
    ln -s $HOME/OneDrive/dev/tiago/pkgs ./build_files/
  fi
fi

sudo /home/logan/dev/singularity-2/bin/singularity build tiago.simg build_files/Singularity.nv