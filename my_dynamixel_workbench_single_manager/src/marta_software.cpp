#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <ros/ros.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "../include/my_dynamixel_workbench_single_manager/marta_dynamixel_controller.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
  // Init ROS node

  ros::init(argc, argv, "marta2_dynamixel_controller");
  //DynamixelController dynamixel_controller;
	MartaDynamixelController marta_controller(*argv);
	marta_controller.InitializeDynamixelController();
	//ros::spinOnce();
  ros::Rate loop_rate(250);
	
	loop_rate.sleep();

	while (ros::ok())
  {
		//std::cout << "Connect: " << 2 << " " << dynamixel_controller.connected(2, 0) << std::endl;
		//std::cout << "Connect: " << 20 << " " << dynamixel_controller.connected(20, 0) << std::endl;
		//printf("Present_position1: %d \n", dynamixel_controller.move_to(5, 0, 100));
		//marta_controller.move_to(4, 0);
		//marta_controller.move_to(1, 0);
		marta_controller.move_to(1, 100);
		marta_controller.move_to(4, 1000);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
