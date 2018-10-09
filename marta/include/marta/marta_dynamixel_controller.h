#ifndef MARTA_DYNAMIXEL_CONTROLLER_H
#define MARTA_DYNAMIXEL_CONTROLLER_H

#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <ros/ros.h>
#include <vector>
#include <map>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include "dynamixel_controller.h"


#define ROUNDF(f,c) (((float)((int)(round((f)*(c))))/(c)))

using namespace cv;
using namespace std;
using namespace dynamixel_controller;

class MartaDynamixelController
{
	private:
		float max_radian;
		float min_radian;

		float convert_Value2Radian(int value, int v_zero, int max_position, int min_position);
		int convert_Radian2Value(float radian, int v_zero, int max_position, int min_position);

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
		bool move(int DXL_ID, int goal_position);
		/* ********* GETTERS  MX-64 ********************/
		float get_present_position(int DXL_ID); // In radian
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
		bool move_to(int DXL_ID, float goal_position); //Goal_Position in Radian
		bool set_torque(int DXL_ID, int torque_status);
		bool set_led_status(int DXL_ID, int led_status);
		bool set_profile_acceleration(int DXL_ID, int profile_acceleration);
		bool set_profile_velocity(int DXL_ID, int profile_velocity);
		bool set_goal_pwm(int DXL_ID, int goal_pwm);
		
};

#endif //MARTA_DYNAMIXEL_CONTROLLER_H
