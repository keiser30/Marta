#include "../include/my_dynamixel_workbench_single_manager/dynamixel_controller.h"

using namespace dynamixel_controller;

DynamixelController::DynamixelController()
{
  // init Service Client
	device[0] = "/dev/ttyUSB0";
	device[1] = "/dev/ttyUSB1";
	port = 0;

	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_head/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_left_arm/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_left_hip/info"));	
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_left_knee/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_left_foot/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_waist/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_right_arm/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_right_hip/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_right_knee/info"));
	dynamixel_info_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("marta_right_foot/info"));

  //dynamixel_command_client_ = node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_head/command");
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_head/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_left_arm/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_left_hip/command"));	
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_left_knee/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_left_foot/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_waist/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_right_arm/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_right_hip/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_right_knee/command"));
	dynamixel_command_client_.push_back(node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("marta_right_foot/command"));
}

DynamixelController::~DynamixelController()
{

}

bool DynamixelController::shutdownDynamixelController(void)
{
  ros::shutdown();
  return true;
}

/******** GET FUNCTION  MX-64******************/
int DynamixelController::get_present_position(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Present_Position;	
	}
	else
		return -1;
}

int DynamixelController::get_firmware_version(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Firmware_Version;	
	}
	else
		return -1;
}

int DynamixelController::get_led_status(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.LED;	
	}
	else
		return -1;
}

int DynamixelController::get_present_current(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Present_Current;	
	}
	else
		return -1;
}

int DynamixelController::get_present_velocity(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Present_Velocity;	
	}
	else
		return -1;
}

int DynamixelController::get_input_voltage(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Present_Input_Voltage;	
	}
	else
		return -1;
}

int DynamixelController::get_present_temperature(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Present_Temperature;	
	}
	else
		return -1;
}

int DynamixelController::get_present_torque(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Torque_Enable;	
	}
	else
		return -1;
}

int DynamixelController::get_profile_acceleration(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Profile_Acceleration;	
	}
	else
		return -1;
}

int DynamixelController::get_profile_velocity(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Profile_Velocity;	
	}
	else
		return -1;
}

int DynamixelController::get_moving_status(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Moving_Status;	
	}
	else
		return -1;
}

int DynamixelController::get_present_pwm(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Present_PWM;	
	}
	else
		return -1;
}


int DynamixelController::get_min_position(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Min_Position_Limit;	
	}
	else
		return -1;

}

int DynamixelController::get_max_position(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Max_Position_Limit;	
	}
	else
		return -1;

}

int DynamixelController::get_acceleration_limit(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Acceleration_Limit;	
	}
	else
		return -1;
}

int DynamixelController::get_velocity_limit(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.mx2_info.Velocity_Limit;	
	}
	else
		return -1;
}

/* ******************GET FUNCTION AX-12 ********************/

int DynamixelController::get_present_positionAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.ax_info.Present_Position;	
	}
	else
		return -1;
}


int DynamixelController::get_firmware_versionAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
		return get_dynamixel_info.response.dynamixel_info.ax_info.Firmware_Version;	
	}
	else
		return -1;
}

int DynamixelController::get_led_statusAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
			return get_dynamixel_info.response.dynamixel_info.ax_info.LED;	
	}
	else
		return -1;
}

int DynamixelController::get_present_velocityAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
			return get_dynamixel_info.response.dynamixel_info.ax_info.Present_Speed;		
	}
	else
		return -1;
}

int DynamixelController::get_input_voltageAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
			return get_dynamixel_info.response.dynamixel_info.ax_info.Present_Voltage;	
	}
	else
		return -1;
}

int DynamixelController::get_present_temperatureAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
			return get_dynamixel_info.response.dynamixel_info.ax_info.Present_Temperature;	
	}
	else
		return -1;
}

int DynamixelController::get_present_torqueAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
			return get_dynamixel_info.response.dynamixel_info.ax_info.Torque_Enable;	
	}
	else
		return -1;
}

int DynamixelController::get_min_positionAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
			return get_dynamixel_info.response.dynamixel_info.ax_info.CW_Angle_Limit;	
	}
	else
		return -1;
}


int DynamixelController::get_max_positionAX(int client, int DXL_ID, int DXL_PORT)
{
	dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	get_dynamixel_info.request.port = (uint8_t)DXL_PORT;
	get_dynamixel_info.request.dxl_id = (uint8_t)DXL_ID;
	if (dynamixel_info_client_[client].call(get_dynamixel_info))
	{
			return get_dynamixel_info.response.dynamixel_info.ax_info.CCW_Angle_Limit;	
	}
	else
		return -1;
}


/***********COMMAND MESSAGE ************/
bool DynamixelController::sendCommandMsg(int client, std::string cmd, uint8_t dxl, std::string addr, int64_t value)
{
	if(client < dynamixel_command_client_.size())
	{
		dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

		set_dynamixel_command.request.command   = cmd;
		set_dynamixel_command.request.addr_name = addr;
		set_dynamixel_command.request.value     = value;
		set_dynamixel_command.request.dxl_id    = dxl;
		set_dynamixel_command.request.port			= port;

		if (dynamixel_command_client_[client].call(set_dynamixel_command))
		{
		  if (!set_dynamixel_command.response.comm_result)
		    return false;
		  else
		    return true;
		}
		else
		{
			return false;
		}
	}
	return false;
}


bool DynamixelController::connected(int client, int DXL_ID, int DXL_PORT){
	port = DXL_PORT;

	if (sendCommandMsg(client, "connect", DXL_ID))
			return true;
	return false;
}


/****** SET FUNCTION ****************/

bool DynamixelController::move_to(int client, int DXL_ID, int DXL_PORT, int goal_position)
{
	port = DXL_PORT;
	if (sendCommandMsg(client, "addr", DXL_ID, "Goal_Position", goal_position))
			return true;
	return false;
}

bool DynamixelController::set_torque(int client, int DXL_ID, int DXL_PORT, int torque_status)
{
	port = DXL_PORT;
	if(torque_status)
	{
		if (sendCommandMsg(client, "torque", DXL_ID, "on", 1))
		{	
			return true;
		}			
		return false;
	}
	else
	{
		if (sendCommandMsg(client, "torque", DXL_ID, "off", 0))
			return true;
		return false;
	}

}

bool DynamixelController::set_led_status(int client, int DXL_ID, int DXL_PORT, int led_status)
{
	port = DXL_PORT;
	if(led_status)
	{
		if (sendCommandMsg(client, "LED", DXL_ID, "on", 1))
		{	
			return true;
		}			
		return false;
	}
	else
	{
		if (sendCommandMsg(client, "LED", DXL_ID, "off", 0))
			return true;
		return false;
	}	
}

bool DynamixelController::set_profile_acceleration(int client, int DXL_ID, int DXL_PORT, int profile_acceleration)
{
	port = DXL_PORT;
	if (sendCommandMsg(client, "addr", DXL_ID, "Profile_Acceleration", profile_acceleration))
			return true;
	return false;
}

bool DynamixelController::set_profile_velocity(int client, int DXL_ID, int DXL_PORT, int profile_velocity)
{
	port = DXL_PORT;
	if (sendCommandMsg(client, "addr", DXL_ID, "Profile_Velocity", profile_velocity))
			return true;
	return false;
}

bool DynamixelController::set_goal_pwm(int client, int DXL_ID, int DXL_PORT, int goal_pwm)
{
	port = DXL_PORT;
	if (sendCommandMsg(client, "addr", DXL_ID, "Goal_PWM", goal_pwm))
			return true;
	return false;
}




/*

bool DynamixelController::controlLoop()
{
	dynamixel_workbench_msgs::GetDynamixelInfo set_dynamixel_info;		
  dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;
	
	get_dynamixel_info.request.port = (uint8_t)(999);
  if (dynamixel_info_client_.call(get_dynamixel_info))
  {
		for(int i = 0; i < PORT_NUM; i++){
		 	if(strcmp(get_dynamixel_info.response.dynamixel_info.load_info.device_name.c_str(), device[i]) == 0){	
				port = (uint8_t)i;
				i = PORT_NUM;
			}
    }                                            
  }	

  char input[128];
  char cmd[80];
  char param[20][30];
	char param2[20][30];
  int num_param = 0;
	int num_param1 = 0;
  char *token;
  bool valid_cmd = false;

  if (kbhit())
  {
    if (getchar() == ENTER_ASCII_VALUE)
    {
      viewManagerMenu();

      printf("[CMD]");
      fgets(input, sizeof(input), stdin);

      char *p;
      if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
      fflush(stdin);

      if (strlen(input) == 0) return false;

      token = strtok(input, " ");

      if (token == 0) return false;

      strcpy(cmd, token);
      token = strtok(0, " ");
      num_param = 0;

      strcpy(param2[num_param++], token);

			token = strtok(0, " ");
			num_param1 = 0;
			while (token != 0)
      {
        strcpy(param[num_param1++], token);
        token = strtok(0, " ");
      }
			
      if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
      {
        viewManagerMenu();
      }
      else if (strcmp(cmd, "info") == 0)
      {

				dynamixel_workbench_msgs::MX2Ext mx64_info;

				get_dynamixel_info.request.dxl_id = (uint8_t)atoi(param2[0]);
        if (dynamixel_info_client_.call(get_dynamixel_info))
        {
          printf("[ID] %u, [Device Name] %s, [Model Name] %s, [Protocol Version] %1.f.0, [BAUD RATE] %ld, [MX] %u\n", 
																																													 get_dynamixel_info.response.dynamixel_info.model_id,
																																													 get_dynamixel_info.response.dynamixel_info.load_info.device_name.c_str(),
                                                                                           get_dynamixel_info.response.dynamixel_info.model_name.c_str(),
                                                                                           get_dynamixel_info.response.dynamixel_info.load_info.protocol_version,
                                                                                           get_dynamixel_info.response.dynamixel_info.load_info.baud_rate,
																																													 mx64_info.Model_Number);
        }
      }
      else if (strcmp(cmd, "exit") == 0)
      {
        if (sendCommandMsg("exit", atoi(param2[0]))){
          shutdownDynamixelController();
				}
        return true;
      }
      else if (strcmp(cmd, "table") == 0)
      {
        if (!sendCommandMsg("table", atoi(param2[0])))
          printf("It didn't load DYNAMIXEL Control Table\n");
      }
      else if (strcmp(cmd, "reboot") == 0)
      {
        if (sendCommandMsg("reboot", atoi(param2[0])))
          printf("It didn't reboot to DYNAMIXEL\n");
      }
      else if (strcmp(cmd, "reset") == 0)
      {
        if (!sendCommandMsg("factory_reset", atoi(param2[0])))
          printf("It didn't factory reset to DYNAMIXEL\n");
      }
      else if (strcmp(cmd, "torque_on") == 0)
      {
        if (!sendCommandMsg("torque", atoi(param2[0]), "on", 1))
          printf("It didn't works\n");
        else
          printf("Torque On");
      }
      else if (strcmp(cmd, "torque_off") == 0)
      {
        if (!sendCommandMsg("torque", atoi(param2[0]), "off", 0))
          printf("It didn't works\n");
        else
          printf("Torque Off");
      }
      else if (strcmp(cmd, "goal") == 0)
      {
				if (atoi(param[0]) >= 5 && atoi(param[0]) <= 1024){
		      if (!sendCommandMsg("addr", atoi(param2[0]), "Goal_Position", atoi(param[0])))
		        printf("It didn't works\n");
		      else
		        printf("Move!!");
					}
				else
						printf("It force o motor\n");
      }
      else if (strcmp(cmd, "id") == 0)
      {
        if (!sendCommandMsg("addr", atoi(param2[0]), "ID", atoi(param[0])))
          printf("It didn't works\n");
      }
      else if (strcmp(cmd, "baud") == 0)
      {
        if (!sendCommandMsg("addr", atoi(param2[0]), "Baud_Rate", atoi(param[0])))
          printf("It didn't works\n");
      }
      else if (strcmp(cmd, "version") == 0)
      {
        if (!sendCommandMsg("addr", atoi(param2[0]), "Protocol_Version", atof(param[0])))
          printf("It didn't works\n");
      }
      else if (num_param == 1)
      {
        if (sendCommandMsg("addr", atoi(param2[0]), cmd, atoi(param[0])))
          printf("It works!!\n");
        else
          printf("It didn't works!!\n");
      }
      else
      {
        printf("Invalid command. Please check menu[help, h, ?]\n");
      }
    }
  }
}*/
