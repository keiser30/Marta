#include "../include/my_dynamixel_workbench_single_manager/marta_dynamixel_controller.h"
#include <sstream>

using namespace cv;
using namespace std;

MartaDynamixelController::MartaDynamixelController(std::string dir)
{
  // init Service Client
	fileDir = "";
	std::string s = dir;
	std::string delimiter = "/";
	size_t pos = 0;
	std::string token;
	while ((pos = s.find(delimiter)) != std::string::npos){
		token = s.substr(0, pos);
		if(token == "devel") 
			break;
		fileDir += token + "/";
		s.erase(0, pos + delimiter.length());
		
	}
	fileDir += "src/my_dynamixel_workbench_single_manager/launch/marta_motor.yaml";

	std::cout << fileDir << std::endl;

	partsOfMarta.push_back("Head");
	partsOfMarta.push_back("LeftArm");
	partsOfMarta.push_back("LeftHip");
	partsOfMarta.push_back("LeftKnee");
	partsOfMarta.push_back("LeftFoot");
	partsOfMarta.push_back("Waist");
	partsOfMarta.push_back("RightArm");
	partsOfMarta.push_back("RightHip");
	partsOfMarta.push_back("RightKnee");
	partsOfMarta.push_back("RightFoot");
}

MartaDynamixelController::~MartaDynamixelController()
{
	for(std::map<int, std::vector<int> >::const_iterator it = map_ids.begin(); it != map_ids.end(); ++it)
	{
		dynamixel_controller.set_torque(it->second[0], it->first, it->second[1], 0);
	}
}

bool MartaDynamixelController::InitializeDynamixelController()
{
	std::vector<std::string> typeMotor;
	typeMotor.push_back("MX64");
	typeMotor.push_back("AX12");


	int port = 0;
	std::string motor = "Motor";
	FileStorage fs(fileDir, FileStorage::READ);
	if(!fs.isOpened())
	{
		return false;
	}	

	int dof = (int) fs["DOF"];

	
	for(int index = 0; index < partsOfMarta.size(); index++)
	{
	//	int index = 0;
		FileNode n = fs[partsOfMarta[index]];
		FileNode aux_n = n["ModelMotor"];	
		std::string modelMotor;
		FileNodeIterator it = aux_n.begin(), it_end = aux_n.end(); // Go through the node
		for (; it != it_end; ++it)
		{
			//std::cout << "Model of Motor: " << (string)*it << std::endl;
			modelMotor = (string)*it;
		}

		for(int i = 0; i < typeMotor.size(); i++)
			if(typeMotor[i] == modelMotor)
			{
				port = i;
				break;
			}

		int number_motor = (int)n["NumberMotor"];
		for(int i = 1; i <= number_motor; i++)
		{
			//std::cout << "Number of motor:" << i << std::endl;
			std::stringstream ss;
			ss << i;
			aux_n = n[motor + ss.str()];
			int DXL_id = (int)aux_n["ID"];
			int id_init = (int)aux_n["Init"];
			int min = (int)aux_n["Min"];
			int max = (int)aux_n["Max"];

			if(dynamixel_controller.connected(index, DXL_id, port))
			{
				std::vector<int> vec;
				vec.push_back(index);
				vec.push_back(port);
				if(port == 0)
				{
					if(min < dynamixel_controller.get_min_position(index, DXL_id, port))
						min = dynamixel_controller.get_min_position(index, DXL_id, port);

					if(max > dynamixel_controller.get_max_position(index, DXL_id, port))
						max = dynamixel_controller.get_max_position(index, DXL_id, port);
				}
				else
				{
					if(min < dynamixel_controller.get_min_positionAX(index, DXL_id, port))
						min = dynamixel_controller.get_min_positionAX(index, DXL_id, port);

					if(max > dynamixel_controller.get_max_positionAX(index, DXL_id, port))
						max = dynamixel_controller.get_max_positionAX(index, DXL_id, port);
				}
			
				vec.push_back(min);
				vec.push_back(max);
				
				map_ids[DXL_id] = vec;
				vec.clear();

				dynamixel_controller.set_torque(index, DXL_id, port, 1);
				move_to(DXL_id, id_init);
				
			}
		}
	}

	for(std::map<int, std::vector<int> >::const_iterator it = map_ids.begin(); it != map_ids.end(); ++it)
	{
		std::cout << "ID: " << it->first;
		std::cout << " client: " << partsOfMarta[it->second[0]];
		std::cout << " port: " << it->second[1];
		std::cout << " min: " << it->second[2];
		std::cout << " max: " << it->second[3] << std::endl;
	}

		printf("Humanoid with DOF %d, but only connect %d \n", dof, map_ids.size());
}

/* ********* GETTERS  MX-64 ********************/
int MartaDynamixelController::get_present_position(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_present_position(it->second[0], DXL_ID, it->second[1]);
	else	
		return dynamixel_controller.get_present_positionAX(it->second[0], DXL_ID, it->second[1]);
}

int MartaDynamixelController::get_firmware_version(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_firmware_version(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_firmware_versionAX(it->second[0], DXL_ID, it->second[1]);
}

int MartaDynamixelController::get_led_status(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_led_status(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_led_statusAX(it->second[0], DXL_ID, it->second[1]);
}

int MartaDynamixelController::get_present_current(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_present_current(it->second[0], DXL_ID, it->second[1]);
	else
		return -1;
}

int MartaDynamixelController::get_present_velocity(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_present_velocity(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_present_velocityAX(it->second[0], DXL_ID, it->second[1]);
}

int MartaDynamixelController::get_input_voltage(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_input_voltage(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_input_voltageAX(it->second[0], DXL_ID, it->second[1]);

}

int MartaDynamixelController::get_present_temperature(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_present_temperature(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_present_temperatureAX(it->second[0], DXL_ID, it->second[1]);
}

int MartaDynamixelController::get_present_torque(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_present_torque(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_present_torqueAX(it->second[0], DXL_ID, it->second[1]);
}

int MartaDynamixelController::get_profile_acceleration(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_profile_acceleration(it->second[0], DXL_ID, it->second[1]);
	else
		return -1;
}

int MartaDynamixelController::get_profile_velocity(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_profile_velocity(it->second[0], DXL_ID, it->second[1]);
	else
		return -1;
}

int MartaDynamixelController::get_moving_status(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)	
		return dynamixel_controller.get_moving_status(it->second[0], DXL_ID, it->second[1]);
	else
		return -1;
}

int MartaDynamixelController::get_present_pwm(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_present_pwm(it->second[0], DXL_ID, it->second[1]);
	else
		return -1;
}

int MartaDynamixelController::get_min_position(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_min_position(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_min_positionAX(it->second[0], DXL_ID, it->second[1]);
}

int MartaDynamixelController::get_max_position(int DXL_ID)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return -1;

	if (it->second[1] == 0)
		return dynamixel_controller.get_max_position(it->second[0], DXL_ID, it->second[1]);
	else
		return dynamixel_controller.get_max_positionAX(it->second[0], DXL_ID, it->second[1]);
}


/* ***************** SET ***********************/
void MartaDynamixelController::shutdownMartaDynamixelController()
{
	for(std::map<int, std::vector<int> >::const_iterator it = map_ids.begin(); it != map_ids.end(); ++it)
	{
		dynamixel_controller.set_torque(it->second[0], it->first, it->second[1], 0);
	}

	dynamixel_controller.shutdownDynamixelController();
}


bool MartaDynamixelController::move_to(int DXL_ID, int goal_position)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return false;
	
	if(goal_position >= it->second[2] && goal_position <= it->second[3])
	{
		if(dynamixel_controller.move_to(it->second[0], DXL_ID, it->second[1], goal_position))
			return true;
	}	

	return false;
}

bool MartaDynamixelController::set_torque(int DXL_ID, int torque_status)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return false;
	
	if(torque_status >= 0 && torque_status <= 1)
	{
		if(dynamixel_controller.set_torque(it->second[0], DXL_ID, it->second[1], torque_status))
			return true;
	}	
	
	return false;
}

bool MartaDynamixelController::set_led_status(int DXL_ID, int led_status)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return false;
	
	if(led_status >= 0 && led_status <= 1)
	{
		if(dynamixel_controller.set_led_status(it->second[0], DXL_ID, it->second[1], led_status))
			return true;
	}	
	
	return false;
}

bool MartaDynamixelController::set_profile_acceleration(int DXL_ID, int profile_acceleration)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return false;
	
	if(it->second[1] == 0)
	{
		if(profile_acceleration >= 0 && profile_acceleration < dynamixel_controller.get_acceleration_limit(it->second[0], DXL_ID, it->second[1]))
		{
			if(dynamixel_controller.set_led_status(it->second[0], DXL_ID, it->second[1], profile_acceleration))
				return true;
		}	
	}
	return false;
}

bool MartaDynamixelController::set_profile_velocity(int DXL_ID, int profile_velocity)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return false;
	
	if(it->second[1] == 0)
	{
		if(profile_velocity >= 0 && profile_velocity < dynamixel_controller.get_velocity_limit(it->second[0], DXL_ID, it->second[1]))
		{
			if(dynamixel_controller.set_led_status(it->second[0], DXL_ID, it->second[1], profile_velocity))
				return true;
		}	
	}
	return false;
}

bool MartaDynamixelController::set_goal_pwm(int DXL_ID, int goal_pwm)
{
	std::map<int, std::vector<int> >::const_iterator it = map_ids.find(DXL_ID);
	if(it == map_ids.end())
		return false;
	
	if(it->second[1] == 0)
	{
		if(goal_pwm >= 0 && goal_pwm < 885)
		{
			if(dynamixel_controller.set_led_status(it->second[0], DXL_ID, it->second[1], goal_pwm))
				return true;
		}	
	}
	return false;	
}


