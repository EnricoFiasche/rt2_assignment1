#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 *  Function used to return a random number between the interval
 *  [N,M]
 *
 *  @param:
 *	M: minimum value allowed for the random number
 *	N: maximum value allowed for the random number 
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 *  Function used by the server to generate the coordinate of a
 *  random goal, x position, y position and theta (the orientation).
 *
 *  @param:
 *	req: request done by a client, where is stored the min and
 *	     max value allowed for x and y position.
 *	res: response of the server, where is stored the three
 *	     random coordinates.
 *
 *  @return: true value when the function finish to store the response.
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 *  Main function that initialize the server.
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
