# The `minimal_turtlebot` ROS Package

This ROS package is for the use in the ECSE 275: *Fundamentals of Robotics* course at Case Western Reserve University.  It is intended to allow students with basic programming skill, but no ROS experience to be able to program a turtlebot.  The program works with physical and simulated turtlebots.

This is accomplished by focusing student work on one function in one file.  The function that the students work on takes as an input a structure with the turtlebot sensor information populated, and pointers to the linear and angular velocity to command the robot (and the sounds to play). 

Students use basic programming skills to interpret, for example, the returns in a LIDAR scan to determine whether to slow down, to stop if a wheel drop sensor is triggered, to back away if a cliff sensor is triggered, and excuse itself a bumper sensor is triggered.

All of these behaviors can be accomplished with basic mathematical operations and logic statements.

## Setup

This package was developed for the turtlebot/turtlebot 2, it is not test on the turtlebot 3.  The turtlebot simulator must be installed.  This is accomplished through typical package manager installation commands for ROS Indigo and ROS Kinetic.  

Only the turtlebot 3 simulator is available from the ROS repositories beginning with ROS Melodic.

An script is included with this package that will ensure the necessary repository packages are installed on the system and will clone, build, and install the remaining dependencies no longer available for the standard ROS package management repositories.

To use this package, first install ROS as described on the [ROS Installation Wiki](http://wiki.ros.org/ROS/Installation).  (Use the `desktop_full` installation.)  If it is not already, ensure that GIT is available on the system (`sudo apt-get install git`).

Next, create a `catkin` workspace where this package will reside and clone this repository into it.

```
# Make a catkin workspace for this package
mkdir -p ~/turtlebot_playground_ws/src
cd ~/turtlebot_playground_ws/src
# Clone this package into the src subdirectory
git clone https://github.com/cwru-eecs-275/minimal_turtlebot.git
```

It is not necessary to have ROS configure yet, the script was developed to start from a clean `desktop-full` installation without even being configured (though that will not hurt).  The next step will ensure that ROS package dependencies that are not part of the ROS installation are installed.  Then repositories for other ROS packages will be cloned, built, and installed.

```
cd ~/turtlebot_playground_ws/src/minimal_turtlebot
./turtlebot_simulation_installation.sh
```

The installation of the full turtlebot simulation can then be tested.

```
# Configure ROS
source /opt/ros/melodic/setup.bash
# Launch the turtlebot simulation
roslaunch turtlebot_gazebo turtlebot_world.launch
```

This should start Gazebo with the turtlebot in a basic world.  The turtlebot does not yet do anything.

This package can then be built and tested.

```
# Get to the correct location in the workspace to build the package
cd ~/turtlebot_playground_ws
# Build the package
catkin_make
# Configure ROS to use this workspace
source devel/setup.bash

# Start the minimal_turtlebot package
roslaunch minimal_turtlebot minimal_turtlebot.launch
```

With any luck, the same simulation environment and turtlebot has shown up.  This time, however, the turtlebot is moving in slow circles.

The environment is now ready.

## Working with the Package

Follow the laboratory document for completing the exercises required for the course.

In general, though, this package allows a user to create a basic program to control a turtlebot.  This program is called a *node* in ROS.  The node that is produced is compatible with the `turtlebot_simulator` package.  It has also been used on control a *real* (vs. simulated) turtlebot.

### Starting the Simulation and Node

The turtlebot simulator may be started directly using the following command:

```
roslaunch turtlebot_gazebo turtlebot_world.launch &
```

The node produced by this package can be started to control the turtlebot in the simulation:

```
rosrun minimal_turtlebot minimal_turtlebot &
```

The simulation and the node can be started simultaneously:

```
roslaunch minimal_turtlebot minimal_turtlebot.launch &
```

In the event another world is launch and the Turtlebot as it functions here is desired in that world, it can be spawned into an existy Gazebo simulation.

```
roslaunch minimal_turtlebot spawn_turtlebot.launch
```

### Editing the Node

The node controlling the turtlebot is structured so that all the "intelligence" is in one function located in the `turtlebot_controller.cpp` file.  

There are many programs available for working with code, however, `gedit` is included with the default Ubuntu installation and is perfectly functional.

```
# A ROS way to change to the base directory of a package
roscd minimal_turtlebot/src

# Open the turtlebot_controller.cpp file for editing
gedit turtlebot_controller.cpp &
```

The file is (or will soon be) well commented to describe how it works.

### Recompiling and Running the Node

This package is developed in the C++ language.  Anytime C++ code is altered, the program must be rebuilt.

```
# The node can only be made from the correct directory
cd ~/turtlebot_playground_ws

# Make the node again
catkin_make
```

Only one instance of this node may be run at one time.  It is possible to see all the currently running nodes.

```
# List the currently running nodes
rosnode list
```

This node will be registered as `/minimal_turtlebot`.  If the node is running, it should be killed before the new version of the node is started.

```
# Kill the /minimal_turtlebot node
rosnode kill /minimal_turtlebot
```

The new node can now be run.

```
# Start the new node
rosrun minimal_turtlebot minimal_turtlebot &
```

### Shutdown

Working with Linux at the command line has some intricacy and it can be useful for those new to it to look for a tutorial on the web.  [Using a Linux Cheat Sheet is also helpful.](https://www.google.com/search?channel=fs&client=ubuntu&q=linux+cheat+sheet+pdf)  (Take your pick.)  The instructions above started all of the ROS elements in the background.  They must be brought to the foreground in order to send close signals to them.

Individual nodes can be killed as described above, however, launch files start many thing.  Shutting down the entire system is a bit more complex.  Given the instructions above, the following will shut the system down.

```
# Bring the minimal_turtlebot node into the foreground
# This allows direct interaction
fg
# CTRL-C sends a signal to the node to end
# The command prompt will return with the node has ended

fg
# In the order above, gedit is now in the foreground
# CTRL-C sends a signal to gedit to close
# It may ask you to save any unsaved changes before it closes

fg
# The roslaunch command should now be in the foreground
# CTRL-C sends a close signal to this
# It may take a little while for everything to close
```

**NB**  Just closing the Gazebo window does not close down the system.  The simulation will continue to run unless it is closed as described.

Like many things, this is not the only way to accomplish these goals, but it is one way that works.  