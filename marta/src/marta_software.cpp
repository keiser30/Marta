#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <ros/ros.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "../include/marta/marta_dynamixel_controller.h"
#include "../include/marta/camera_sensor.h"
#include "../include/marta/imu_sensor.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
  // Init ROS node

  ros::init(argc, argv, "marta_dynamixel_controller");
	MartaDynamixelController marta_controller(*argv);
	marta_controller.InitializeDynamixelController();
	CameraSensor camera;
	ImuSensor imu;
	ros::spinOnce();
  ros::Rate loop_rate(250);
	
	loop_rate.sleep();

	while (ros::ok())
  {
		std::cout << "Width " << camera.get_width() << std::endl;
		std::cout << "IMU " << imu.get_orientation_quaternion(0)[0] << std::endl;

		camera.show_image();

		marta_controller.move_to(1, 100);
		marta_controller.move_to(4, 1000);
		
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
