#!/bin/bash

cd "${BASH_SOURCE%/*}/" || exit # cd into the bundle and use relative paths

sudo singularity shell \
  --writable \
  --bind=build_files/pkgs:/pkgs \
  --bind=build_files/scripts:/scripts \
  tiago