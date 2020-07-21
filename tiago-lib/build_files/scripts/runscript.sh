#!/bin/bash

export PS1="\[$(tput bold)\]\[\033[38;5;226m\]\A\[$(tput sgr0)\]\[\033[38;5;227m\]:\[$(tput sgr0)\]\[$(tput sgr0)\]\[\033[38;5;15m\] \[$(tput bold)\]\[$(tput sgr0)\]\[\033[38;5;87m\]\w\[$(tput sgr0)\]\[$(tput sgr0)\]\[\033[38;5;15m\] \[$(tput bold)\]\[$(tput sgr0)\]\[\033[38;5;82m\][\$?]\[$(tput sgr0)\]\[$(tput sgr0)\]\[\033[38;5;15m\]\\$ \[$(tput sgr0)\]"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

VENV=/code/python-env
if [ -f "$VENV/bin/activate" ]; then
  source $VENV/bin/activate
fi

source /opt/pal/erbium/setup.bash
if [ -f "/tiago_ws/devel/setup.bash" ]; then
  source /tiago_ws/devel/setup.bash
fi

alias rosm="export ROS_MASTER_URI=http://tiago-37c:11311"
alias rosi="export ROS_IP=\$(hostname -I | grep -oEi '10\.68\.[0-9]+\.[0-9]+')"
alias srcd="source /tiago_ws/devel/setup.bash"
alias pycharm="bash -i -c \"/opt/pycharm-community-2019.2.1/bin/pycharm.sh >/dev/null 2>&1 &\""
alias jarp="/code/PetriNetPlans/Jarp/jarp.sh >/dev/null 2>&1 &"

# link the PNP code
if [ ! -d "/tiago_ws/src/pnp_ros" ]; then
  ln -s /code/PetriNetPlans/PNPros/ROS_bridge/pnp_ros /tiago_ws/src
  ln -s /code/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs /tiago_ws/src
  ln -s /code/PetriNetPlans/PNPros/ROS_bridge/pnp_rosplan /tiago_ws/src
fi

cd /tiago_ws

if [ ! -d "$HOME/.ros/rosdep/sources.cache" ]; then
  rosdep update
fi

# TODO: use this to keep the .vscode and .idea files up to date with new python packages
#       rather than polluting the PYTHONPATH variable
# update PYTHONPATH for vscode/pycharm
# update_python_path () {
  # for PKG_DIR in /tiago_ws/src/*/ ; do
  #   if [ -f "${PKG_DIR}/setup.py" ]; then
  #     PKG_DIR_SRC="${PKG_DIR}src"
  #     IFS=':' read -r -a ARRAY <<< "${PYTHONPATH}"
  #     NEW_ARRAY=()
  #     for ITEM in "${ARRAY[@]}"
  #     do
  #         [[ $ITEM != "${PKG_DIR_SRC}" ]] && NEW_ARRAY+=($ITEM)
  #     done
  #     PYTHONPATH=$(IFS=':' ; echo "${NEW_ARRAY[*]}")
  #     export PYTHONPATH="${PKG_DIR_SRC}:${PYTHONPATH}"
  #   fi
  # done
# }
# update_python_path

if [ -f "$HOME/.bashrc" ]; then
  source $HOME/.bashrc
fi

export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
export GAZEBO_PLUGIN_PATH=/tiago_ws/src/elevator/build/devel/lib

rossavemap () {
    rosservice call /pal_map_manager/save_map "directory: '$1'"
}

roschangemap () {
    rosservice call /pal_map_manager/change_map "input: '$1'"
}

alias robot="rosm && rosi && clear && echo 'Connected to TIAGo'"
alias robotrviz="rosrun rviz rviz -d `rospack find tiago_2dnav`/config/rviz/navigation.rviz"
alias rosmode-map="rosservice call /pal_navigation_sm \"input: 'MAP'\""
alias rosmode-loc="rosservice call /pal_navigation_sm \"input: 'LOC'\""
alias rosmove="rosrun key_teleop key_teleop.py"
alias rosloc="rosservice call /global_localization \"{}\""
alias rosclearmap="rosservice call /move_base/clear_costmaps \"{}\""
alias rosaction="rosrun actionlib axclient.py /play_motion"
alias stop="rosm && rosi && rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}"
alias roshelp="echo 'robot robotrviz rosaction rosmode-map rosmode-loc rosmove rosloc rosclearmap stop rossavemap roschangemap'"

