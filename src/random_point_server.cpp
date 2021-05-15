#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/random_position.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition; /** RandomPosition service */
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

	class PositionService : public rclcpp::Node
	{
		public:
			/**
			 *  Position service constructor that creates a new service
			 */
			PositionService(const rclcpp::NodeOptions & options) : Node("random_position_server", options){
				// creating a new service for RandomPosition
				service_ = this->create_service<RandomPosition>("/position_server",std::bind(&PositionService::myrandom, this, _1, _2, _3));
			}
		
		private:
		
			/**
 			  *  Function used to return a random number between the interval
 			  *  [N,M]
			  *
			  *  @param:
			  *		M: minimum value allowed for the random number
			  *		N: maximum value allowed for the random number 
			  *
			  *  @return: random value between M and N
 			  */
			double randMToN(double M, double N)
			{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

			/**
			 *  Function used by the server to generate the coordinate of a
			 *  random goal, x position, y position and theta (the orientation).
			 *
			 *  @param:
			 *  request_header: to avoid warnings.
			 *	req: request done by a client, where is stored the min and
			 *	     max value allowed for x and y position.
			 *	res: response of the server, where is stored the three
			 *	     random coordinates.
			 *
			 *  @return: true value when the function finish to store the response.
			 */
			bool myrandom (const std::shared_ptr<rmw_request_id_t> request_header,
						   const std::shared_ptr<RandomPosition::Request> req,
						   const std::shared_ptr<RandomPosition::Response> res){
				(void)request_header;
				res->x = randMToN(req->x_min, req->x_max);
				res->y = randMToN(req->y_min, req->y_max);
				res->theta = randMToN(-3.14, 3.14);
				return true;
			}
			
			/** variable used to manage the server */
			rclcpp::Service<RandomPosition>::SharedPtr service_;
	};
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::PositionService) 

