// Common mechanism to prevent recursive including of file
#ifndef TURTLEBOT_CONTROLLER_H
#define TURTLEBOT_CONTROLLER_H

#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h> 
#include <angles/angles.h>

#include <std_msgs/Float64.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <kobuki_msgs/WheelDropEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/SensorState.h>


struct turtlebotInputs
{
	// time
	uint64_t nanoSecs;

	//wheel drop states
	uint8_t leftWheelDropped;
	uint8_t rightWheelDropped; 
	
	//bumper states 
	uint8_t leftBumperPressed; 
	uint8_t centerBumperPressed;
	uint8_t rightBumperPressed;
	
	//color and depth images
	sensor_msgs::Image colorImage;  
	sensor_msgs::Image depthImage;  
	
	//cliff states
	uint8_t sensor0State; 
	uint8_t sensor1State; 
	uint8_t sensor2State; 
	
	//laserscan data
	float ranges[640];
	float minAngle; 
	float maxAngle; 
	float angleIncrement; 
	int numPoints; 
	
	//imu data 
	float linearAccelX; 
	float linearAccelY; 
	float linearAccelZ; 
	
	float angularVelocityX; 
	float angularVelocityY; 
	float angularVelocityZ;
	
	float orientationX;
	float orientationY;
	float orientationZ;
	float orientationW; 
	
	//batt voltage
	float battVoltage; 
	
	//odom
	
	// Position
	float x;
	float y;
	float z;
	// Orientation
	float qw;
	float qx;
	float qy;
	float qz; 
};

struct goalPose_t {
	// Position
	float pose_x;
	float pose_y;
	float pose_z;
	// Orientation
	float quat_w;
	float quat_x;
	float quat_y;
	float quat_z;
};

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel);

extern goalPose_t goalPose;

#define GET_GOAL(x, y) {x = goalPose.pose_x; y = goalPose.pose_y;}

#define SOUND_OFF kobuki_msgs::Sound::OFF
#define SOUND_RECHARGE kobuki_msgs::Sound::RECHARGE
#define SOUND_BUTTON kobuki_msgs::Sound::BUTTON
#define SOUND_ERROR kobuki_msgs::Sound::ERROR
#define SOUND_CLEANINGSTART kobuki_msgs::Sound::CLEANINGSTART
#define SOUND_CLEANINGEND kobuki_msgs::Sound::CLEANINGEND

static double (*shortest_angular_distance)(double, double) = angles::shortest_angular_distance;

#endif // TURTLEBOT_CONTROLLER_H
