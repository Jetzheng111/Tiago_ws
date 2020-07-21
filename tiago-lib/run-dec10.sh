#!/bin/bash

mkdir -p home workdir tiago_ws/src

singularity run \
  --contain \
  --nv \
  --bind /etc/localtime \
  --bind /run \
  --bind build_files/scripts:/scripts \
  --home home:${HOME} \
  --workdir workdir \
  --bind tiago_ws:/tiago_ws \
  /vol/scratch/sclmd/tiago.simg

