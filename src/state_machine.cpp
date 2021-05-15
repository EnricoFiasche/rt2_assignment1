#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

	class StateMachine : public rclcpp::Node{

		public:
			
			/**
			 *  StateMachine constructor that a server, which allows the node to read the command sent by the user,
			 *  and two, one to get a random target point and one to communicate with go_to_point, which move
			 *  the robot in order to reach the goal.
			 */
			StateMachine(const rclcpp::NodeOptions & options) : Node("state_machine", options){
				// creating the service to manage the 
				service_ = this->create_service<Command>("/user_interface",std::bind(&StateMachine::user_interface, this, _1, _2, _3));
				
				goto_client_ = this->create_client<Position>("/go_to_point");
				while(!goto_client_->wait_for_service(std::chrono::seconds(3))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "position client interrupted");
						return ;
					}
					RCLCPP_INFO(this->get_logger(), "waiting for position service to appear...");
				}
				
				random_client_ = this->create_client<RandomPosition>("/position_server");
				while(!random_client_->wait_for_service(std::chrono::seconds(3))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "random position client interrupted");
						return ;
					}
					RCLCPP_INFO(this->get_logger(), "waiting for random service to appear...");
				}
			}
			
			/**
			 *  Function used to make request to the RandomPosition server and to send the
			 *  goal position to go_to_point.
			 *  If the value of start is true, the function makes a request to the RandomPosition server
			 *  in order to get the next goal, then sends it to go_to_point and waits the result. 
			 */
			void fsm(){
				auto rp = std::make_shared<RandomPosition::Request>();
				rp->x_max = 5.0;
			   	rp->x_min = -5.0;
			   	rp->y_max = 5.0;
			   	rp->y_min = -5.0;
			   	
			   	goal_pos = std::make_shared<Position::Request>();
			   	
			   	if (this->start){
			   	
			   		using ServiceResponseFuture = rclcpp::Client<RandomPosition>::SharedFuture;
				   	auto response_received = [this] (ServiceResponseFuture future) {
				   		// getting the goal position from the response
				   		goal_pos->x = future.get()->x;
				   		goal_pos->y = future.get()->y;
				   		goal_pos->theta = future.get()->theta;

						std::cout << "\nGoing to the position: x = " << goal_pos->x << " y = " << goal_pos->y << " theta = " << goal_pos->theta << std::endl;
						
						using ServicePosFuture = rclcpp::Client<Position>::SharedFuture;
				   		auto response_pos = [this] (ServicePosFuture futurepos){
				   			// if the target is reached print a success message
				   			if(futurepos.get()->ok){
				   				std::cout << "Goal Reached!" << std::endl;
				   				fsm();
				   			}
				   		};
				   		// send an async request
				   		auto future_request_pos = goto_client_->async_send_request(goal_pos,response_pos);
					};
			   		// send an async request
			   		auto future_request = random_client_->async_send_request(rp, response_received);
			   	}
			}
		
		private:
			
			/**
			 *  Function of the user_interface server used to modify the value of
			 *  the variable "start", based on the command received by the client. 
			 *
			 *  @param:
			 *	request_header: to avoid warnings.
			 *	req: request sent by the client, which could be "start"
			 *	     or "stop".
			 *	res: response of the server, which is "true" when it finishes
			 *	    to modify the start value.
			 *
			 *  @return true when it finishes to modify the start value
			 */
			bool user_interface(const std::shared_ptr<rmw_request_id_t> request_header,
							 	const std::shared_ptr<Command::Request> req,
							 	const std::shared_ptr<Command::Response> res){
				
				(void)request_header;
				
				if (req->command == "start"){
					this->start = true;
					this->fsm(); // call the function finite state machine
				}
				else 
					this->start = false;

				return true;
			}
			
			bool start = false; /** bool variable that stores the command sent by the user */
			rclcpp::Service<Command>::SharedPtr service_; /** service used to manage user_interface */
			rclcpp::Client<Position>::SharedPtr goto_client_; /** go_to_point client */
			rclcpp::Client<RandomPosition>::SharedPtr random_client_; /** random position client */
			std::shared_ptr<Position::Request> goal_pos; /** goal position */
	};
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
