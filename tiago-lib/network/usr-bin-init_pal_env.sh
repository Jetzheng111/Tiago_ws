#!/bin/bash

##########################################################################################
# Logan added this 09-08-2019 because pyasn1 could not be updated in the system packages #
##########################################################################################
if test -f /home/pal/python-env/bin/activate; then
    echo -e "\033[33mUsing virtual-env in /home/pal/python-env\033[0m"
    source /home/pal/python-env/bin/activate
fi

# pick the most recent pal distro
export PAL_DISTRO=`ls /opt/pal/ | sort | tail -n 1`

# check if there is a deployed_ws available
if test -f /home/pal/deployed_ws/setup.bash; then
    echo -e "\033[33mUsing packages and overlays in /home/pal/deployed_ws\033[0m"
    source /home/pal/deployed_ws/setup.bash
elif test -f /home/pal/logan_ws/devel/setup.bash; then
    echo -e "\033[33mUsing packages and overlays in /home/pal/logan_ws/devel\033[0m"
    source /home/pal/logan_ws/devel/setup.bash
elif test -f /opt/pal/$PAL_DISTRO/setup.bash; then
    source /opt/pal/$PAL_DISTRO/setup.bash
else
    echo -e "\033[31mERROR: Could not prepare $PAL_DISTRO distro environment. setup.bash not found\033[0m"
fi

# if control is defined use it for the ros master uri
cat /etc/hosts | grep control > /dev/null
if [ "$?" = "0" ]; then
    export ROS_MASTER_URI=http://control:11311
fi

PATH="/opt/pal/$PAL_DISTRO/lib/pal_debian_utils/:$PATH"