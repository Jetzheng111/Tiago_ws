Requirements
1. 3GB absolute minimum storage (I recommend at least 10GB)
2. some form of Linux (MacOS and Windows may be supported using Vagrant to run Singularity)
2
Installation
1. Download either the tiago.sif or tiago-hybrid.sif container file from RobotClub/files tab on our
yammer group (tiago-hybrid for hybrid GPU) (Inside the tiago-lib file)
2. Install sylabs singularity.
3. Clone tiago-lib from the github.
4. Move the downloaded .sif file into your cloned tiago-lib folder.(done)
5. Change the last line of run.sh to ”tiago.sif” in tiago-lib
3
Shell into the container
go to tiago-lib in terminal and run the bash script:
• ./run.sh for user-space
sif containers are read-only, so you will need to rebuild tiago.sif if root access is required for things like
installing software. (catkin workspaces still have user write access in the .sif)
4
Writable container (root access)
1. singularity build --sandbox tiago-rw tiago.sif
2. Change the last line of run.sh to ”tiago-rw” in tiago-lib
3. Change the last line of write.sh to ”tiago-rw” in tiago-lib
4. ./run.sh for user-space (catkin workspaces are writable)
5. ./write.sh to install software that require root access
6. Delete tiago.sif if you would like to free up space



clone object detection yolo from git to the workspace for OpenCV4 pre-built for the container.
(2GB, includes YOLOv3 weights) (Done)


References
Please refer to tiago-lib and the handbook for other useful commands
