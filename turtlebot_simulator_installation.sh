#!/bin/bash

# Setup ROS
if [ ${#ROS_PACKAGE_PATH} -lt 1 ]
then
    ROS_VERS=($(ls /opt/ros 2>/dev/null)) 
    if [ $? -ne 0 ]; then
        echo "Install ROS first"
    elif [ ${#ROS_VERS[@]} -gt 1 ]
    then
        select ver in ${ROS_VERS[@]}; do echo $ver; break; done
    elif [ ${#ROS_VERS[@]} -eq 0 ]
    then
        if [ -d /opt/ros ]
        then
            echo "The ROS installation is corrupted."
        else
            echo "ROS has not been installed."
        fi
        exit
    else
        ver=${ROS_VERS}
    fi

    if [ -f /opt/ros/$ver/setup.bash ]; then
        source /opt/ros/$ver/setup.bash
    else
        echo "The ROS setup file is not there!!"
        exit
    fi


    if [ ${#ROS_PACKAGE_PATH} -lt 1 ]
    then
        echo "ROS appears misconfigured.  It may not be installed or there may be another issue."
    fi
else 
    ver=$ROS_DISTRO
fi

if [ $ver == 'noetic' ]; then
    echo "This script does not yet work with ROS Noetic."
    exit
fi

# Get a bunch of dependencies
sudo apt-get -y install pyqt5-dev-tools ros-melodic-joy ros-melodic-kobuki-driver ros-melodic-kobuki-msgs ros-melodic-kobuki-dock-drive ros-melodic-depthimage-to-laserscan ros-melodic-laptop-battery-monitor ros-melodic-ecl-threads ros-melodic-ecl-exceptions ros-melodic-ecl-sigslots ros-melodic-ecl-streams ros-melodic-ecl-threads ros-melodic-ecl-exceptions ros-melodic-ecl-sigslots ros-melodic-ecl-time ros-melodic-ecl-geometry ros-melodic-ecl-linear-algebra ros-melodic-yocs-cmd-vel-mux ros-melodic-yocs-controllers ros-melodic-yocs-velocity-smoother ros-melodic-yocs-virtual-sensor 

if [ $? -ne 0 ]; then
    echo "Some required packages were not installed"
    exit
fi


# Set up a temporaroy catkin workspace
mkdir -p /tmp/turtlebot_ws/src
cd /tmp/turtlebot_ws/src

gitrepos=( "https://github.com/turtlebot/turtlebot_simulator.git" "https://github.com/turtlebot/turtlebot.git" "https://github.com/yujinrobot/kobuki.git" "https://github.com/turtlebot/turtlebot_apps.git" "https://github.com/turtlebot/turtlebot_msgs.git" "https://github.com/yujinrobot/kobuki_desktop.git")
failedgits=("")

# Clone the necessary repositories
for repo in ${gitrepos[@]}; do
    if [ -d `basename -s .git ${repo}` ]; then
    echo "Repository already cloned"
    else
        git clone ${repo}
        if [ $? -neq 0 ]; then
            echo "Problem cloning ${repo}"
            failedgits+=(${repo})
        fi
    fi
done

if [ ${#failedgits} -gt 0 ]; then
    echo "Cloning of the following repos failed:"
    for repo in ${failedgits[@]}; do 
        echo "${repo}"
    done
    exit
fi

# Make everything
cd /tmp/turtlebot_ws
catkin_make

# The make process failed, take care to see why
if [ $? -gt 0 ]; then
    echo "The catkin_make process did not succeed."
    exit  
fi

# Install everything
source devel/setup.bash
sudo -- /bin/bash -c "source /opt/ros/$ver/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/$ver install" 

if [ $? -eq 0 ]; then
    echo "Turtlebot simulator should be installed now"
    echo "Run (or rerun) source /opt/ros/${ver}/setup.bash"
else
    echo "Installation appears to have failed."
fi