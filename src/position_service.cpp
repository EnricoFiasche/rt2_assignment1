/**
 * \file position_service.cpp
 * \brief It is a simple server that generates three random number.
 * \author Enrico Fiasche' S4482512
 * \version 0.1
 * \date 28/05/2021
 * 
 * \details
 * 
 * Advertised Service: <BR>
 *	/position_server
 * 
 * Description:
 * 
 * This node is a simple server that generates three random number.
 * This node uses the service message RandomPosition which has as request
 * the minimum and maximum value allowed for the x and y position, and
 * it has as response the three coordinates of the random goal
 * (x,y posiion and the orientation theta).
 */

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * 	\brief it returns three random number
 *
 *  \param M: minimum value allowed for the random number
 *	\param N: maximum value allowed for the random number 
 *
 *  \return random value between M and N
 * 
 * 	Function used to return a random number between the interval [N,M]
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 *  \brief It generets random values for the positions
 *
 *  \param req: request done by a client, where is stored the min and
 *	     max value allowed for x and y position.
 *	\param res: response of the server, where is stored the three
 *	     random coordinates.
 *
 *  \return true value when the function finish to store the response.
 * 
 * 	Function used by the server to generate the coordinate of a
 *  random goal, x position, y position and theta (the orientation).
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 *  \param argc: number of argument passed as parameter
 *  \param argv: values passed as parameter
 * 
 *  \return 0 when the program ends
 * 
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
