/**
 * \file state_machine.cpp
 * \brief Main node that manages the whole simulation
 * \author Enrico Fiasche' S4482512
 * \version 0.1
 * \date 28/05/2021
 * 
 * \details
 * 
 * Publishes to <BR>
 * 	/stateResult
 * 
 * Client: <BR>
 *	/user_interface
 *  
 * Action client: <BR>
 * 	/position_server
 * 
 * Description:
 * 
 * state_machine is a node used to communicate with all the other nodes.
 * This one has a simple client, on /position_server, in order to receive
 * the next goal, a server, where receives a request by the user
 * and an action client, which is used to send new goals and to check if
 * a target is reached. Before start searching a new goal the state machine 
 * checks if there is a Command request from a client, then if the user
 * command is "start", the node make a request to the server receiving
 * a goal between -5.0 and 5.0, regarding x and y position, and the
 * theta orientation, then the state machine sends this goal to the
 * server using the action client and waits for a result, which
 * could be a "success" in case of goal reached and a "failure" in
 * case of goal canceled. The node publishes this result on a specific topic.
 */
#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "rt2_assignment1/PositionAction.h"
#include "std_msgs/Int32.h"

bool start = false; /**< variable used to read the command sent by the user */


/**
 *  \brief Function used to change the value of variable start
 *   
 *	\param req: request sent by the client, which could be "start" or "stop".
 *	
 *	\param res: response of the server, which is "true" when it finishes 
 * 				to modify the start value.
 *
 *  \return always true when it finishes to modify the start value
 * 
 *  Function of the user_interface server used to modify the value of
 *  the variable "start", based on the command received by the client.
 */
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){

    if(req.command == "start")
    	start = true;

    else if(req.command == "stop")
    	start = false;

    return true;
}


/**
 * 	\brief main function that initialize and manage the simulation
 * 
 *  \param argc: number of argument passed as parameter
 *  \param argv: values passed as parameter
 * 
 *  \return 0 when the program ends
 * 
 *  Main function that creates the service "user_interface", a
 *  client, which is used to receive a random target position, and
 *  an action client, which is used to send a goal and to check if
 *  the goal is reached.
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
	ros::Publisher results_pub = n.advertise<std_msgs::Int32>("/stateResult", 1000);
   
	actionlib::SimpleActionClient<rt2_assignment1::PositionAction> action_client("go_to_point", true);

	while(!action_client.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	   
	rt2_assignment1::RandomPosition rp;
	rp.request.x_max = 5.0;
	rp.request.x_min = -5.0;
	rp.request.y_max = 5.0;
	rp.request.y_min = -5.0;

	std_msgs::Int32 state;
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
			
			action_client.waitForResult();
			if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				std::cout << "Target reached!" << std::endl;
				state.data = 1;
				results_pub.publish(state);
			}
			else if(action_client.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
				std::cout << "Goal canceled" << std::endl;
				state.data = 0;
				results_pub.publish(state);
			}
			
			ros::Duration(0.5).sleep();
		}
   }
   
   return 0;
}
