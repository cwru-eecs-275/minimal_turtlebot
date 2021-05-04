#include "minimal_turtlebot/turtlebot_controller.h"


void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
	//Place your code here! you can access the left / right wheel 
	//dropped variables declared above, as well as information about
	//bumper status. 
	
	//outputs have been set to some default values. Feel free 
	//to change these constants to see how they impact the robot. 

	*vel = 0.2; // Robot forward velocity in m/s
	//0.7 is max and is a lot
	*ang_vel = 0.2;  // Robot angular velocity in rad/s
	//0.7 is max and is a lot 
  

	float x, y;

	GET_GOAL(x, y);

	// Send a debug message throttled to 1 Hz.
	ROS_INFO_THROTTLE(1, "Goal Pose X = %2.2f, Y = %2.2f", x, y);
	// Send a debug message every loop.
	// ROS_INFO("Goal Pose X = %2.2f, Y = %2.2f", x, y);
  
	//here are the various sound value enumeration options
	//SOUND_OFF
	//SOUND_RECHARGE
	//SOUND_BUTTON
	//SOUND_ERROR
	//SOUND_CLEANINGSTART
	//SOUND_CLEANINGEND 
	*soundValue = SOUND_OFF;

}

