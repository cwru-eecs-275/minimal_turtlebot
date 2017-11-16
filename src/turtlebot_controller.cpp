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
  
	*soundValue = 0;
  
	ROS_INFO("X: %f:", turtlebot_inputs.x); 
	ROS_INFO("Y: %f:", turtlebot_inputs.y); 
	ROS_INFO("Z: %f:", turtlebot_inputs.z_angle); 
	ROS_INFO("Omega: %f:", turtlebot_inputs.orientation_omega); 
	//here are the various sound value enumeration options
	//soundValue.OFF
	//soundValue.RECHARGE
	//soundValue.BUTTON
	//soundValue.ERROR
	//soundValue.CLEANINGSTART
	//soundValue.CLEANINGEND 

}

