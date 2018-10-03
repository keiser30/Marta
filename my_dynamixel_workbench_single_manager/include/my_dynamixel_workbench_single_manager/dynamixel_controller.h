#ifndef MY_DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H
#define MY_DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H

#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <ros/ros.h>
#include <vector>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "message_header.h"

namespace dynamixel_controller
{
#define ESC_ASCII_VALUE             0x1b
#define SPACEBAR_ASCII_VALUE        0x20
#define ENTER_ASCII_VALUE           0x0a
#define PORT_NUM										2

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher

  // ROS Topic Subscriber

  // ROS Service Server

  // ROS Service Client
  std::vector<ros::ServiceClient> dynamixel_info_client_;
  std::vector<ros::ServiceClient> dynamixel_command_client_;


  // Dynamixel Workbench Parameters
	char* device[PORT_NUM];
	uint8_t port;

 public:
  DynamixelController();
  ~DynamixelController();
  
	/******** GET FUNCTION ******************/
	int get_present_position(int client, int DXL_ID, int DXL_PORT);
	int get_firmware_version(int client, int DXL_ID, int DXL_PORT);
	int get_led_status(int client, int DXL_ID, int DXL_PORT);
	int get_present_current(int client, int DXL_ID, int DXL_PORT);
	int get_present_velocity(int client, int DXL_ID, int DXL_PORT);
	int get_input_voltage(int client, int DXL_ID, int DXL_PORT);
	int get_present_temperature(int client, int DXL_ID, int DXL_PORT);
	int get_present_torque(int client, int DXL_ID, int DXL_PORT);
	int get_profile_acceleration(int client, int DXL_ID, int DXL_PORT);
	int get_profile_velocity(int client, int DXL_ID, int DXL_PORT);
	int get_moving_status(int client, int DXL_ID, int DXL_PORT);
	int get_present_pwm(int client, int DXL_ID, int DXL_PORT);
	int get_min_position(int client, int DXL_ID, int DXL_PORT);
	int get_max_position(int client, int DXL_ID, int DXL_PORT);
	int get_acceleration_limit(int client, int DXL_ID, int DXL_PORT);
	int get_velocity_limit(int client, int DXL_ID, int DXL_PORT);

	int get_present_positionAX(int client, int DXL_ID, int DXL_PORT);
	int get_firmware_versionAX(int client, int DXL_ID, int DXL_PORT);
	int get_led_statusAX(int client, int DXL_ID, int DXL_PORT);
	int get_present_velocityAX(int client, int DXL_ID, int DXL_PORT);
	int get_input_voltageAX(int client, int DXL_ID, int DXL_PORT);
	int get_present_temperatureAX(int client, int DXL_ID, int DXL_PORT);
	int get_present_torqueAX(int client, int DXL_ID, int DXL_PORT);
	int get_min_positionAX(int client, int DXL_ID, int DXL_PORT);
	int get_max_positionAX(int client, int DXL_ID, int DXL_PORT);


	/****** SET FUNCTION ****************/
	bool connected(int client, int DXL_ID, int DXL_PORT);
	bool move_to(int client, int DXL_ID, int DXL_PORT, int goal_position);
	bool set_torque(int client, int DXL_ID, int DXL_PORT, int torque_status);
	bool set_led_status(int client, int DXL_ID, int DXL_PORT, int led_status);
	bool set_profile_acceleration(int client, int DXL_ID, int DXL_PORT, int profile_acceleration);
	bool set_profile_velocity(int client, int DXL_ID, int DXL_PORT, int profile_velocity);
	bool set_goal_pwm(int client, int DXL_ID, int DXL_PORT, int goal_pwm);

	bool shutdownDynamixelController();

 private:
  bool sendCommandMsg(int client, std::string cmd, uint8_t dxl = 1, std::string addr = "", int64_t value = 0);

};
}

#endif //MY_DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H
