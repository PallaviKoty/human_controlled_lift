// Copyright (c) 2014 Mohit Shridhar, David Lee

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <iostream>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <std_msgs/Int8.h>

#include <elevator_plugin/ControlGroup.h>
#include <elevator_plugin/AddGroup.h>
#include <elevator_plugin/DeleteGroup.h>
#include <elevator_plugin/ListGroups.h>
#include <elevator_plugin/OpenCloseDoors.h>
#include <elevator_plugin/SetVelDoors.h>

#define CONTROL_GROUP_NAME "keyboard_op_control_group"

#define DEFAULT_ELEV_SPEED 1.5
#define DEFAULT_ELEV_FORCE 100

#define DEFAULT_SLIDE_SPEED 1.0
#define DEFAULT_FLIP_SPEED 1.57

enum ControlType {DOOR, ELEVATOR};

class KeyboardOp
{
	public:
		ros::NodeHandle rosNode;
		ros::ServiceClient add_group_client, delete_group_client, list_groups_client;
		ros::ServiceClient open_close_doors_client, set_vel_doors_client, target_floor_elev_client, set_elev_props_client, open_close_elev_doors_client;

		ControlType type;
		std::string groupName;
		bool isGroupInitialized;
		// start();

		ros::Subscriber sub = rosNode.subscribe("door_command", 1000, &KeyboardOp::executeDoorServices, this);

		elevator_plugin::OpenCloseDoors openDoorsCall;
		elevator_plugin::OpenCloseDoors closeDoorsCall;
		elevator_plugin::SetVelDoors setVelDoorsCall;

	public:
		KeyboardOp(ros::NodeHandle &nh)
		{
			rosNode = nh;
			rosNode = ros::NodeHandle("");

			setupClientServices();
			initVars();
		}

		void initVars()
		{
			type = DOOR;
		}

		void setupClientServices()
		{
			add_group_client = rosNode.serviceClient<elevator_plugin::AddGroup>("model_dynamics_manager/add_control_group");
			delete_group_client = rosNode.serviceClient<elevator_plugin::DeleteGroup>("model_dynamics_manager/delete_control_group");
			list_groups_client = rosNode.serviceClient<elevator_plugin::ListGroups>("model_dynamics_manager/list_groups");

			open_close_doors_client = rosNode.serviceClient<elevator_plugin::OpenCloseDoors>("model_dynamics_manager/doors/open_close");
			set_vel_doors_client = rosNode.serviceClient<elevator_plugin::SetVelDoors>("model_dynamics_manager/doors/set_vel");

		}

		bool setControlType(char input[])
		{
			std::string inputStr(input);

			if (boost::iequals(inputStr, "door")) {
				type = DOOR;
				setActiveUnits();
				return true;
			} 
			return false;
		}

		void setActiveUnits()
		{
			char input[]="3,4";
			std::vector<uint32_t> activeList = parseActiveList(input);
			
			// Delete previous group if already initialized. Note: IGNORE the warning produced during initialization about delete service failing
			elevator_plugin::DeleteGroup deleteSrv;
			deleteSrv.request.group_name = CONTROL_GROUP_NAME;
			delete_group_client.call(deleteSrv);

			// Add new group with the desired units
			elevator_plugin::AddGroup addSrv;
			addSrv.request.group.group_name = CONTROL_GROUP_NAME;
			addSrv.request.group.type = "door";
			addSrv.request.group.active_units = activeList;
			add_group_client.call(addSrv);

			printDoorControls();

			isGroupInitialized = true;
		}

		void printDoorControls()
		{
		    std::cout << "\n-----------------\nDoor Controls:\nPress 'Enter' after each input.\n'q' to quit.\n'o' to open doors\n'c' to close doors" << std::endl;
		}

	    std::vector<uint32_t> parseActiveList(char input[])
	    {
	      std::string active_list_str(input);
	      std::vector<uint32_t> active_list;

	      // parse csv-style input (also remove whitespace):
	      std::string::iterator end_pos = std::remove(active_list_str.begin(), active_list_str.end(), ' ');
	      active_list_str.erase(end_pos, active_list_str.end());

	      std::istringstream ss(active_list_str);
	      std::string token;

	      while (std::getline(ss, token, ',')) {
	      	try {
	        	active_list.push_back(atoi(token.c_str()));
	      	} catch (...) {
	      		std::cout << "Invalid active list. Exiting.." << std::endl;
	      		std::exit(EXIT_SUCCESS);
	      	}
	      }

	      return active_list;
	    }

		void initialize()
		{
			isGroupInitialized = false;

			char input[] = "door";

			while (!setControlType(input)) {
				std::cout << "Invalid type. Options: 'door' or 'elevator'" << std::endl;
			}

			setupCallTemplates();
		}

		void setupCallTemplates()
		{
			// DOOR based services:
			openDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			openDoorsCall.request.state = true;

			closeDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			closeDoorsCall.request.state = false;

			setVelDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			setVelDoorsCall.request.lin_x = DEFAULT_SLIDE_SPEED;
			setVelDoorsCall.request.lin_y = DEFAULT_SLIDE_SPEED;
			setVelDoorsCall.request.ang_z = DEFAULT_FLIP_SPEED;

		}

		void start()
		{
			initialize();
			bool initialSetup = true;

				if (initialSetup){
					char input[] = "door";

					if (setControlType(input)) {
						initialSetup = false;						
					}
				}
		}

		void executeDoorServices(const std_msgs::Int8::ConstPtr& msg)
		{
			std::cout << "Inside executeDoorServices, I heard " << msg->data << std::endl;
			// std::string inputStr(input);

			if(msg->data == 1) {
					open_close_doors_client.call(openDoorsCall);
			}
			if(msg->data == 0) {
					open_close_doors_client.call(closeDoorsCall);
			}
			else{
					std::cout << "Unknown command" << std::endl;
			};
		}

		float parseFloat(std::string input)
		{
		    std::string::iterator end_pos = std::remove(input.begin(), input.end(), ' ');
		    input.erase(end_pos, input.end()); 

		    return atof(input.c_str());	
		}
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "keyboard_op_model_dynamics_control");
  ros::NodeHandle nh;
  KeyboardOp controller(nh);
  
  controller.start();
  ros::spin();
}