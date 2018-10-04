/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef MY_DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H
#define MY_DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H

#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <ros/ros.h>
#include <vector>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "message_header.h"

namespace single_dynamixel_controller
{
#define ESC_ASCII_VALUE             0x1b
#define SPACEBAR_ASCII_VALUE        0x20
#define ENTER_ASCII_VALUE           0x0a
#define PORT_NUM										2

class SingleDynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher

  // ROS Topic Subscriber

  // ROS Service Server

  // ROS Service Client
  ros::ServiceClient dynamixel_info_client_;
  ros::ServiceClient dynamixel_command_client_;

  // Dynamixel Workbench Parameters
	char* device[PORT_NUM];
	uint8_t port;

 public:
  SingleDynamixelController();
  ~SingleDynamixelController();
  void viewManagerMenu(void);
  bool controlLoop(void);

 private:
  bool initSingleDynamixelController();
  bool shutdownSingleDynamixelController();

  int getch(void);
  int kbhit(void);

  bool sendCommandMsg(std::string cmd, uint8_t dxl = 1, std::string addr = "", int64_t value = 0);

};
}

#endif //MY_DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H
