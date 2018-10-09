#include "../include/marta/imu_sensor.h"



ImuSensor::ImuSensor()
{
	subData_.push_back(node_handle_.subscribe("/marta_imu_upper/imu/data", 1, &ImuSensor::imuSensorUpperDataMsgCallback, this));	
	subMag_.push_back(node_handle_.subscribe("/marta_imu_upper/imu/mag", 1, &ImuSensor::imuSensorUpperMagMsgCallback, this));
	subRpy_.push_back(node_handle_.subscribe("/marta_imu_upper/imu/rpy", 1, &ImuSensor::imuSensorUpperRpyMsgCallback, this));
	subTemp_.push_back(node_handle_.subscribe("/marta_imu_upper/imu/temperature", 1, &ImuSensor::imuSensorUpperTempMsgCallback, this));
/*
	subData_.push_back(node_handle_.subscribe("/marta_imu_lower/imu/data", 1, &ImuSensor::imuSensorLowerDataMsgCallback, this));	
	subMag_.push_back(node_handle_.subscribe("/marta_imu_lower/imu/mag", 1, &ImuSensor::imuSensorLowerMagMsgCallback, this));
	subRpy_.push_back(node_handle_.subscribe("/marta_imu_lower/imu/rpy", 1, &ImuSensor::imuSensorLowerRpyMsgCallback, this));
	subTemp_.push_back(node_handle_.subscribe("/marta_imu_lower/imu/temperature", 1, &ImuSensor::imuSensorLowerTempMsgCallback, this));
*/
	std::vector<float> vec3;
	vec3.push_back(0.0);
	vec3.push_back(0.0);
	vec3.push_back(0.0);
	std::vector<float> vec4;
	vec4.push_back(0.0);
	vec4.push_back(0.0);
	vec4.push_back(0.0);
	vec4.push_back(0.0);
	for(int i = 0; i < IMU_NUMBER; i++)
	{
		orientationQuaternion.push_back(vec4);
		angularVelocity.push_back(vec3);
		linearAcceleration.push_back(vec3);
		magnetometer.push_back(vec3);
		orientationRadians.push_back(vec3);
		temperature.push_back(0.0);
	}
}


ImuSensor::~ImuSensor()
{
	//Destructor
}


void ImuSensor::imuSensorDataMsg(const sensor_msgs::Imu::ConstPtr& msg, int i)
{
	orientationQuaternion[i][0] = msg->orientation.x;
	orientationQuaternion[i][1] = msg->orientation.y;
	orientationQuaternion[i][2] = msg->orientation.z;
	orientationQuaternion[i][3] = msg->orientation.w;

	angularVelocity[i][0] = msg->angular_velocity.x;
	angularVelocity[i][1] = msg->angular_velocity.y;
	angularVelocity[i][2] = msg->angular_velocity.z;	

	linearAcceleration[i][0] = msg->linear_acceleration.x;
	linearAcceleration[i][1] = msg->linear_acceleration.y;
	linearAcceleration[i][2] = msg->linear_acceleration.z;
}


void ImuSensor::imuSensorUpperDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imuSensorDataMsg(msg, 0);
}

void ImuSensor::imuSensorLowerDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imuSensorDataMsg(msg, 1);
}


void ImuSensor::imuSensorMagMsg(const geometry_msgs::Vector3Stamped::ConstPtr& msg, int i)
{
	magnetometer[i][0] = msg->vector.x;
	magnetometer[i][1] = msg->vector.y;
	magnetometer[i][2] = msg->vector.z;
}


void ImuSensor::imuSensorUpperMagMsgCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	imuSensorMagMsg(msg, 0);
}


void ImuSensor::imuSensorLowerMagMsgCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	imuSensorMagMsg(msg, 1);
}

	
void ImuSensor::imuSensorRpyMsg(const geometry_msgs::Vector3Stamped::ConstPtr& msg, int i)
{
	orientationRadians[i][0] = msg->vector.x;
	orientationRadians[i][1] = msg->vector.y;
	orientationRadians[i][2] = msg->vector.z;
}

void ImuSensor::imuSensorUpperRpyMsgCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	imuSensorRpyMsg(msg, 0);
}

void ImuSensor::imuSensorLowerRpyMsgCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	imuSensorRpyMsg(msg, 1);
}

void ImuSensor::imuSensorTempMsg(const std_msgs::Float32::ConstPtr& msg, int i)
{
	temperature[i] = msg->data;
}

void ImuSensor::imuSensorUpperTempMsgCallback(const std_msgs::Float32::ConstPtr& msg)
{
	imuSensorTempMsg(msg, 0);
}

void ImuSensor::imuSensorLowerTempMsgCallback(const std_msgs::Float32::ConstPtr& msg)
{
	imuSensorTempMsg(msg, 1);
}

std::vector<float> ImuSensor::get_orientation_quaternion(int index)
{	
	std::vector<float> vec;
	if(index < IMU_NUMBER)
		return orientationQuaternion[index];
	return vec;
}

std::vector<float> ImuSensor::get_angular_velocity(int index)
{
	std::vector<float> vec;
	if(index < IMU_NUMBER)
		return angularVelocity[index];
	return vec;
}

std::vector<float> ImuSensor::get_linear_acceleration(int index)
{
	std::vector<float> vec;
	if(index < IMU_NUMBER)
		return linearAcceleration[index];
	return vec;
}

std::vector<float> ImuSensor::get_magnetometer(int index)
{
	std::vector<float> vec;
	if(index < IMU_NUMBER)
		return magnetometer[index];
	return vec;
}

std::vector<float> ImuSensor::get_orientation_radians(int index)
{
	std::vector<float> vec;
	if(index < IMU_NUMBER)
		return orientationRadians[index];
	return vec;
}

float ImuSensor::get_temperature(int index)
{
	if(index < IMU_NUMBER)
		return temperature[index];
	return -1.0;
}



