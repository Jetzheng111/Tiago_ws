#!/bin/bash

cd $(dirname $0)

mkdir -p ../pkgs

# mount the iso with the below code:
# sudo mount -o loop -t iso9660 pal-robotics-software_leeds_8.23_erbium.iso /media/iso

input="pkgs_files.txt"
while IFS= read -r pkg
do
  find /media/iso -name "$pkg" -type f -exec cp {} ../pkgs/ \;
done < "$input"

(cd ../pkgs && dpkg-scanpackages . /dev/null 2>/dev/null | gzip -9c > Packages.gz)