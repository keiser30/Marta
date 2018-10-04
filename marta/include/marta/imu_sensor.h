#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "ros/ros.h"
#include "math.h"
#include "time.h"
#include "sys/time.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <dirent.h>
#include <iostream>
#include <vector>
#include <sstream>


#define IMU_NUMBER 1

class ImuSensor
{
 private:
  // ROS NodeHandle
  	ros::NodeHandle node_handle_;

		std::vector<ros::Subscriber > subData_;
		std::vector<ros::Subscriber > subMag_; 
		std::vector<ros::Subscriber > subRpy_;
		std::vector<ros::Subscriber > subTemp_;  


		std::vector<std::vector<float> > orientationQuaternion;
		std::vector<std::vector<float> > angularVelocity;
		std::vector<std::vector<float> > linearAcceleration;	
		std::vector<std::vector<float> > magnetometer;
		std::vector<std::vector<float> > orientationRadians;		
		std::vector<float>	temperature;
		

	private:
		void imuSensorDataMsg(const sensor_msgs::Imu::ConstPtr& msg, int i);
		void imuSensorMagMsg(const geometry_msgs::Vector3Stamped::ConstPtr& msg, int i);			
		void imuSensorRpyMsg(const geometry_msgs::Vector3Stamped::ConstPtr& msg, int i);
		void imuSensorTempMsg(const std_msgs::Float32::ConstPtr& msg, int i);

		void imuSensorDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
		void imuSensorMagMsgCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);			
		void imuSensorRpyMsgCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
		void imuSensorTempMsgCallback(const std_msgs::Float32::ConstPtr& msg);

	public:
		ImuSensor();
  	~ImuSensor();

		/* *********** GET ********************/
		std::vector<float> get_orientation_quaternion(int index); 
		std::vector<float> get_angular_velocity(int index);
		std::vector<float> get_linear_acceleration(int index);
		std::vector<float> get_magnetometer(int index);
		std::vector<float> get_orientation_radians(int index);

		float get_temperature(int index);


};


#endif //IMU_SENSOR_H

