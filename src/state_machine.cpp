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
			
			StateMachine(const rclcpp::NodeOptions & options) : Node("state_machine", options){
				// creating the service to manage the 
				service_ = this->create_service<Command>("/user_interface",std::bind(&StateMachine::user_interface, this, _1, _2, _3));
				
				pos_client_ = this->create_client<Position>("/go_to_point");
				while(!pos_client_->wait_for_service(std::chrono::seconds(3))){
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
			
			void fsm(){
				auto rp = std::make_shared<RandomPosition::Request>();
				rp->x_max = 5.0;
			   	rp->x_min = -5.0;
			   	rp->y_max = 5.0;
			   	rp->y_min = -5.0;
			   	
			   	pos = std::make_shared<Position::Request>();
			   	
			   	if (this->start){
			   	
			   		using ServiceResponseFuture = rclcpp::Client<RandomPosition>::SharedFuture;
				   	auto response_received = [this] (ServiceResponseFuture future) {
				   		pos->x = future.get()->x;
				   		pos->y = future.get()->y;
				   		pos->theta = future.get()->theta;

						std::cout << "\nGoing to the position: x = " << pos->x << " y = " << pos->y << " theta = " << pos->theta << std::endl;
					};
			   			
			   		auto future_request = random_client_->async_send_request(rp, response_received);
	   				
	   				//pos->x = x; pos->y = y; pos->theta = theta;
			   		
			   		using ServicePosFuture = rclcpp::Client<Position>::SharedFuture;
			   		auto response_pos = [this] (ServicePosFuture futurepos){
			   			if(futurepos.get()->ok){
			   				std::cout << "Goal Reached!" << std::endl;
			   				fsm();
			   			}
			   		};
			   			
	   				auto future_request_pos = pos_client_->async_send_request(pos,response_pos);
			   	}
			}
		
		private:
			
			bool user_interface(const std::shared_ptr<rmw_request_id_t> request_header,
							 	const std::shared_ptr<Command::Request> req,
							 	const std::shared_ptr<Command::Response> res){
				
				(void)request_header;
				
				if (req->command == "start"){
					this->start = true;
					this->fsm();
				}
				else 
					this->start = false;
				
				
				std::cout << "COMMAND RECEIVED " << start << std::endl;

				return true;
			}
			
			bool start = false;
			rclcpp::Service<Command>::SharedPtr service_;
			rclcpp::Client<Position>::SharedPtr pos_client_;
			rclcpp::Client<RandomPosition>::SharedPtr random_client_;
			//double x,y,theta;
			std::shared_ptr<Position::Request> pos;
	};
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
