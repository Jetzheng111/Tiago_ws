#!/bin/bash

cd "${BASH_SOURCE%/*}/" || exit # cd into the bundle and use relative paths

mkdir -p home workdir tiago_ws/src

singularity run \
  --contain \
  $(if lspci | grep -qi nvidia; then echo ' --nv'; else echo ''; fi) \
  --bind=/etc/hosts \
  --bind=/etc/localtime \
  --bind=/run \
  --bind=/proc \
  --bind=/dev \
  --bind=/sys \
  --bind=build_files/scripts:/scripts \
  --home=home:${HOME} \
  --workdir=workdir \
  --bind=tiago_ws:/tiago_ws \
  --add-caps=CAP_NET_RAW \
  tiago
