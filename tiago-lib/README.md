# Cheatsheet

```shell
# sim movement
rosrun key_teleop key_teleop.py
# sim mapping
roslaunch tiago_2dnav_gazebo tiago_mapping.launch
# sim navigation
roslaunch tiago_2dnav_gazebo tiago_navigation.launch
# robot map and navigation
rosrun rviz rviz -d `rospack find tiago_2dnav`/config/rviz/navigation.rviz
# save map
rosservice call /pal_map_manager/save_map "directory: '<NAME>'"
# change map -> loc or vice versa
rosservice call /pal_navigation_sm "input: 'MAP'"
rosservice call /pal_navigation_sm "input: 'LOC'"
# change map
rosservice call /pal_map_manager/change_map "input: 'elevator_floor_6'"
# launch custom world
roslaunch gazebo_ros empty_world.launch
GAZEBO_RESOURCE_PATH=/opt/pal/erbium/share/pal_gazebo_worlds GAZEBO_MODEL_PATH=/opt/pal/erbium/share/pal_gazebo_worlds/models roslaunch gazebo_ros empty_world.launch world_name:=worlds/small_office.world
# relay a topic
rosrun topic_tools relay /throttle_filtering_points/filted_points /obstacle_cloud
# visualize voxel_grid
rosrun costmap_2d costmap_2d_markers voxel_grid:=<your_voxel_grid_topic> visualization_marker:=<your_visualization_marker_topic>
# camera calibration
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/xtion/rgb/image_raw camera:=/xtion/rgb --no-service-check
rosnode kill /pal_topic_monitor
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/xtion/ir/image camera:=/xtion/ir --no-service-check
# topic graph
rosrun rqt_graph rqt_graph
# logging
rqt_console
# keywords
WAKE_WORD="hey robot" OUTPUT_DIRECTORY=/tiago_ws/src/wake_word/keywords SYSTEM=linux TARGET_SYSTEM=linux MACHINE=x86_64 sh -c 'cd /code/Picovoice/porcupine/ && tools/optimizer/${SYSTEM}/${MACHINE}/pv_porcupine_optimizer -r resources/optimizer_data -w "${WAKE_WORD}" -p ${TARGET_SYSTEM} -o ${OUTPUT_DIRECTORY}'
 
```

# Read
http://wiki.ros.org/Names
http://wiki.ros.org/StyleGuide
http://wiki.ros.org/PyStyleGuide
http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown

# Setup

```shell
# choose based on gfx card (nv=nvidia, hybrid=nvidia+intel, nonnv=rest)
./build.sh {--nv, --hybrid, --nonnv}

# enter the container
./run.sh

# fetch dependent projects
cd /tiago_ws/src
wstool up

# link the startup for gazebo
ln -s /tiago_ws/src/tiago_startup ~/.pal/pal_startup

# turn off pal super strict flags
# remove old build files if coming from catkin_make: rm -rf /tiago_ws/build /tiago_ws/devel /tiago_ws/logs
# remove all the CMAKE_CXX_FLAGS changes from at the bottom of the readme
catkin config --cmake-args -DDISABLE_PAL_FLAGS=ON
# catkin config --cmake-args -DDISABLE_PAL_FLAGS=ON -DCMAKE_CXX_FLAGS="-Wno-error -Wall -Wextra -Wconversion -Werror=conversion-null"

# build the workspace
cd /tiago_ws
catkin build
```

edit `/etc/hosts` to include `10.68.0.1  tiago-37c`

(nodelet ssh fail)
```shell
rm -rf ~/.ssh/known_hosts
ssh -oHostKeyAlgorithms='ssh-rsa' pal@tiago-37c
```

# Add a project

```shell
cd /tiago_ws/src
catkin_create_pkg camera_throttler_nodelets nodelet nodelet_topic_tools sensor_msgs pluginlib rospy
catkin_create_pkg lasr_pcl roscpp pcl_ros std_msgs geometry_msgs sensor_msgs message_generation
catkin_create_pkg lasr_door rospy roscpp pcl_ros std_msgs geometry_msgs sensor_msgs message_generation
catkin_create_pkg lasr_moveit rospy roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface std_msgs geometry_msgs sensor_msgs pluginlib tf2_ros message_generation 
cd camera_throttler_nodelets
git init
# make project on gitlab
git remote add origin git@gitlab.com:sensible-robots/camera_throttler_nodelets.git
git add .
git commit -m "feat: first commit"
git push -u origin master
cd /tiago_ws/src
wstool set camera_throttler_nodelets --git git@gitlab.com:sensible-robots/camera_throttler_nodelets.git
wstool set lasr_pcl --git git@gitlab.com:sensible-robots/lasr_pcl.git
wstool set lasr_door --git git@gitlab.com:sensible-robots/lasr_door.git
wstool set lasr_moveit --git git@gitlab.com:sensible-robots/lasr_moveit.git
# commit and push .rosinstall to https://gitlab.com/sensible-robots/tiago-lib
```

# Network Setup

There is a PAL package that manages the network setup (poorly) of Tiago, which allows you to edit it from the WebCommander interface. It is terribly limited in what configuration it allows you to do, so I took things into my own hands.

1. DO NOT FORGET that Tiago is in `read-only` mode by default. All changes outside of the pal user directories will be reset on reboot (Handbook - 9.3 File system)

```shell
ssh root@tiago-37c
rw
chroot /ro
# <make changes>
exit
ro
```

2. There was a spurious `auto eth0` in the `/etc/network/interfaces` file which caused the networking service to fail, meaning the WebCommander setup didn't work.

3. There is a script at `/usr/sbin/network_control.sh` which monitors for network changes and resets everything. Very frustrating and painful to find. Pending feedback from PAL about how to disable the service that calls this file, I did the following:

```shell
cp /usr/sbin/network_control.sh /usr/sbin/network_control.sh.bak
echo "echo \"*********** network_control.sh has been disabled ***********\"" > /usr/sbin/network_control.sh
```

4. I redesigned Tiago's network architecture like so:

```
    Robotics_WIFI                       TIAGO                        LOGAN_ROBOT_LAB
+--------------------+     +---------------------------+     +------------------------------+
|                    |     |                           |     |                              |
|          router    +--+  |                   +-+eth0 +--+  |     +-+NAT/iptables+-+       |
|       192.168.0.50 |  |  |                   +       |  |  |     +                +       |
|         +          |  |  |    wlan0         br0      |  +--+    eth0             eth1     +-->Internet
|         +          |  +--+ 192.168.0.30  10.68.0.1   |  |  | 10.68.0.200     dhcp x.x.x.x |
| logan-laptop:wlan0 |     |                   +       |  |  |                              |
|   192.168.0.31     |     |                   +-+eth1 +--+  |                              |
|                    |     |                           |     |                              |
+--------------------+     +---------------------------+     +------------------------------+
```

5. Much attemptings, long suffering. The routing and NAT/iptables looks as follows

      1. Routing

         | Computer        | Destination     | Gateway       | Interface   |
         |-----------------|-----------------|---------------|-------------|
         | LOGAN_ROBOT_LAB | 192.168.0.0/24  | 10.68.0.1     | eth0        |
         | LOGAN_LAPTOP    | 10.68.0.0/24    | 192.168.0.30  | wlan0       |

         ```shell
         LOGAN_ROBOT_LAB$> ip route add 192.168.0.0/24 via 10.68.0.1 dev eth0

         # Only needed if LOGAN_LAPTOP connected to internet some other way with a default route (usb, lan, etc)
         # Will also need to update default route on Tiago to point to 192.168.0.30
         LOGAN_LAPTOP$> ip route add 10.68.0.0/24 via 192.168.0.30 dev wlan0
         ```
      
      2. NAT/iptables

         ```shell
         # Set net.ipv4.ip_forward=1 in /etc/sysctl.conf
         LOGAN_ROBOT_LAB$> sysctl -p /etc/sysctl.conf
         LOGAN_ROBOT_LAB$> iptables -t nat -A POSTROUTING -s 10.68.0.0/24 -o eth1 -j MASQUERADE

         # Will need to do similar for LOGAN_LAPTOP if connected to internet
         ```

      3. DON'T FORGET TO CHANGE THE DEFAULT ROUTE ON TIAGO!!!!

         ```shell
         ip route del default
         ip route add default via 10.68.0.200
         ```

      3. Make permanent by copying, replacing device name and making executable (`chmod +x`):

         >`./network/etc-network-if-up.d-tiago-routes` --> `/etc/network/if-up.d/tiago-routes`
         >
         >`./network/etc-network-if-post-down.d-tiago-routes` --> `/etc/network/if-post-down.d/tiago-routes`
         
      4. Same as above for laptops/devices on Robotics_WIFI using files with `-wifi` suffix

6. Copy the network interface config from `./network/etc-network-interfaces` --> `/etc/network/interfaces`

7. Add the following and/or whatever nameservers you want to `/etc/resolvconf/resolv.conf.d/base`:
   >nameserver 129.11.159.84
   >
   >nameserver 129.11.159.85

8. \[WORK-IN-PROGRESS\] Still need to reconfigure the DHCP settings so that Tiago can become a hotspot again, although he only connects at 802.11g speeds when he is the hotspot, might try see if there is a setting somewhere to change this

9. When get a new iso from PAL, run the isolinux command on it as in https://wiki.syslinux.org/wiki/index.php?title=Doc/isolinux

# Old fix for catkin build errors

## fix PAL strict errors by adding the following 3 lines AFTER 'find_package(catkin REQUIRED COMPONENTS **)'
 - tiago_ws/src/rosplan/rosplan_knowledge_base/CMakeLists.txt
 - tiago_ws/src/rosplan/rosplan_planning_system/CMakeLists.txt

```cmake
string(REPLACE " -Werror=return-type" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE " -Werror=conversion-null" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE " -Werror=type-limits" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
```


https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/
```shell
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/home/pal/logan_ws/src/lasr_object_detection_yolo/opencv4 \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D CMAKE_POSITION_INDEPENDENT_CODE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=~/code/opencv_contrib/modules \
	-D PYTHON_EXECUTABLE=/home/pal/python-env/bin/python \
	-D BUILD_EXAMPLES=OFF ..

make -j$(grep -c ^processor /proc/cpuinfo)
make install
```

https://github.com/BVLC/caffe/issues/2171

(1) edit CMakeCache.txt
(2) Change:
CMAKE_CXX_FLAGS:STRING=-fPIC
(3) re-compile and install.
(4) power through the SegmentationFaults