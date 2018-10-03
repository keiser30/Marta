#ifndef MY_DYNAMIXEL_WORKBENCH_SINGLE_MARTA_DYNAMIXEL_CONTROLLER_H
#define MY_DYNAMIXEL_WORKBENCH_SINGLE_MARTA_DYNAMIXEL_CONTROLLER_H

#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <ros/ros.h>
#include <vector>
#include <map>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "dynamixel_controller.h"

using namespace cv;
using namespace std;
using namespace dynamixel_controller;

class MartaDynamixelController
{
	public:
		DynamixelController dynamixel_controller;
		
		std::string fileDir;
		std::vector<std::string> partsOfMarta;

		/*	This map container:
					Key: ID
					Client: 
					Port: PORT_ID
					Min:
					Max
		*/
		std::map<int, std::vector<int> > map_ids;

		MartaDynamixelController(std::string dir);
		~MartaDynamixelController();

		
		bool InitializeDynamixelController();

		/* ********* GETTERS  MX-64 ********************/
		int get_present_position(int DXL_ID);
		int get_firmware_version(int DXL_ID);
		int get_led_status(int DXL_ID);
		int get_present_current(int DXL_ID);
		int get_present_velocity(int DXL_ID);
		int get_input_voltage(int DXL_ID);
		int get_present_temperature(int DXL_ID);
		int get_present_torque(int DXL_ID);
		int get_profile_acceleration(int DXL_ID);
		int get_profile_velocity(int DXL_ID);
		int get_moving_status(int DXL_ID);
		int get_present_pwm(int DXL_ID);
		int get_min_position(int DXL_ID);
		int get_max_position(int DXL_ID);



		/* ************ SET FUNCTION ****************/
		void shutdownMartaDynamixelController();
		bool move_to(int DXL_ID, int goal_position);
		bool set_torque(int DXL_ID, int torque_status);
		bool set_led_status(int DXL_ID, int led_status);
		bool set_profile_acceleration(int DXL_ID, int profile_acceleration);
		bool set_profile_velocity(int DXL_ID, int profile_velocity);
		bool set_goal_pwm(int DXL_ID, int goal_pwm);
		
};

#endif //MY_DYNAMIXEL_WORKBENCH_SINGLE_MARTA_DYNAMIXEL_CONTROLLER_H
