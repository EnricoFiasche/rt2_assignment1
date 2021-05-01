#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "rt2_assignment1/PositionAction.h"

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    
    std::cout << "Changing START value: " << start << std::endl;
    return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
	actionlib::SimpleActionClient<rt2_assignment1::PositionAction> action_client("go_to_point", true);

	//std::cout << "WAIT FOR ACTION SERVER" << std::endl;
	while(!action_client.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	   
	rt2_assignment1::RandomPosition rp;
	rp.request.x_max = 5.0;
	rp.request.x_min = -5.0;
	rp.request.y_max = 5.0;
	rp.request.y_min = -5.0;
   
	while(ros::ok()){
		ros::spinOnce();
		
		if (start){
			client_rp.call(rp);
   		
			rt2_assignment1::PositionGoal goal;
			goal.x = rp.response.x; 
			goal.y = rp.response.y;
			goal.theta = rp.response.theta;
   		
			action_client.sendGoal(goal);
   		
			std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
		
			while(!action_client.waitForResult(ros::Duration(3.0)) && start)
				std::cout << "START IS " << start << std::endl;
				
			if(!start){
				std::cout << "CANCELLING THE GOAL" << std::endl;
				action_client.cancelGoal();
			}
			
			if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				std::cout << "Target reached!" << std::endl;
			}
		}
   }
   
   return 0;
}
