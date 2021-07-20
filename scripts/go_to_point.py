#! /usr/bin/env python

## @package rt2_assignment2
# \file go_to_point.py
# \brief This node drive the robot to the desired position received
# \author Enrico Fiasche' S4482512
# \version 0.1
# \date 28/07/2021
#
# \details
#
# Subscribes to: <BR>
#	/odom
#	/consts
#
# Publishes to: <BR>
#	/cmd_vel
#
# Action server: <BR>
# 	/go_to_point
#
# This node is an action server used to reach a given goal.
# This node use a publisher, which publishes the velocity of the robot
# through the cmd_vel topic, and a subscriber, which checks the position
# of the robot through the odom topic. If a new goal is set, this node
# fix the yaw angle in order to be aligned with the goal, then the robot
# can go straight towards the target point and finally, when the robot
# is near the desired position, it can fix the final yaw angle in order
# to have the same orientation of the given goal. During these operations,
# the node checks continuously if the goal is canceled in order to stop
# the robot.
#

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import rt2_assignment1.msg
import actionlib
import math

## point used to store the actual position
position_ = Point()

## yaw angle set 0 as default
yaw_ = 0

## position set 0 as default
position_ = 0

## state set 0 as default (fix yaw angle)
state_ = 0

## publisher used to pub to the topic cmd_vel
pub_ = None

## yaw precision allowed (+/- 20 degree)
yaw_precision_ = math.pi / 9

## yaw precision2 allowed (+/- 2 degree)
yaw_precision_2_ = math.pi / 90

## distance precision from goal allowed
dist_precision_ = 0.1

## angular proportional constant
kp_a = -3.0 

## linear proportional constant
kp_d = 0.2

## upper bound angular velocity
ub_a = 0.6

## lower bound angular velocity
lb_a = -0.5

## upper bound linear velocity
ub_d = 0.6

## action server used to manage go_to_point
action_server = None

def clbk_consts(msg_consts):
	##
	#	\brief update constants callback
	#
	#	\param msg_consts: contains the new linear and angular constants 
	#
	#	Callback function used to update the linear and angular constants
	#	used during the motion.
	#
	
	global kp_d, kp_a;
	
	kp_d = msg_consts.linear.x
	kp_a = msg_consts.angular.z
	
def clbk_odom(msg):
	##
	#	\brief odometry callback
	#	
	#	\param msg: contains the position and orientation of the robot
	#
	#	Callback function used to check the actual position of the 
	#	robot through the odom message received.
	
	global position_
	global yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]


def change_state(state):
	##
	#	\brief change the state of the robot.
	#	
	#	\param state: new machine state
	#
	#	Function used to change the state of the go_to_point service.
	#
	
	global state_
	state_ = state
	print ('State changed to [%s]' % state_)


def normalize_angle(angle):
	##
	#	\brief normalize the angle
	#	
	#	\param angle: angle to normalize
	#		
	#	\return: angle: angle, passed as parameter, normalized
	#
	#	Function used to normalize the angle
	#
	
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def fix_yaw(des_pos):
	##
	#	\brief fix the actual yaw angle
	#
	#	\param des_pos: actual goal position used to fix the yaw
	#				   angle of the robot.
	#
	# 	Function used to fix the actual yaw angle in order to allow
	#	the robot to go straight to the goal.
	#	If the yaw angle is correct (checking also the precision)
	#	the robot changes the state in "go_straight_ahead".
	#
	
	global kp_a;
	
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = normalize_angle(desired_yaw - yaw_)
	#rospy.loginfo(err_yaw)
	twist_msg = Twist()
	
	if math.fabs(err_yaw) > yaw_precision_2_:
		twist_msg.angular.z = kp_a*err_yaw
		if twist_msg.angular.z > ub_a:
			twist_msg.angular.z = ub_a
		elif twist_msg.angular.z < lb_a:
			twist_msg.angular.z = lb_a
	pub_.publish(twist_msg)
	
	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_2_:
		change_state(1)


def go_straight_ahead(des_pos):
	##
	#	\brief the robot goes straight ahead to the goal position.
	#	
	#	\param des_pos: goal position used to allow the robot to
	#			       reach the desired position
	#
	#	Function used to go straight ahead to the goal position.
	#	If the robot reaches the goal position (checking also the
	#	precision) changes the state in "fix the final yaw angle".		
	#
	
	global kp_d, kp_a;
	
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
						pow(des_pos.x - position_.x, 2))
	err_yaw = normalize_angle(desired_yaw - yaw_)
	#rospy.loginfo(err_yaw)

	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = kp_d#*0.3
		if twist_msg.linear.x > ub_d:
			twist_msg.linear.x = ub_d

		twist_msg.angular.z = kp_a*err_yaw
		pub_.publish(twist_msg)
	else: # state change conditions
		change_state(2)

	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
		change_state(0)

def fix_final_yaw(des_yaw):
	##
	#	\brief fix the final yaw angle.
	#	
	#	\param des_yaw: desired yaw final angle to be reached 
	#
	#	Function used to fix the final yaw angle of the robot in
	#	order to reach the final goal position and orientation.
	#	If the angle is aligned with the des_yaw it changes the state
	#	in "Target reached".
	#
	
	global kp_a;
	
	err_yaw = normalize_angle(des_yaw - yaw_)
	#rospy.loginfo(err_yaw)
	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_2_:
		twist_msg.angular.z = kp_a*err_yaw
		if twist_msg.angular.z > ub_a:
			twist_msg.angular.z = ub_a
		elif twist_msg.angular.z < lb_a:
			twist_msg.angular.z = lb_a
	pub_.publish(twist_msg)
    # state change conditions
	if math.fabs(err_yaw) <= yaw_precision_2_:
		#print ('Yaw error: [%s]' % err_yaw)
		change_state(3)
        
def done():
	##
	#	Function used to stop the robot publishing 0 velocity on the
	#	topic cmd_vel.
	#
	
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub_.publish(twist_msg)
    
def go_to_point(goal):
	##
	#	\brief
	#	\param goal: goal position and orientation received.
	#
	#	Function used by the action server to manage the behaviour
	#	of the robot.
	#	The function, for each cycle, checks if the goal was canceled
	#	and checks the actual state in order to perform the correct
	#	function.

	global action_server, state_
	
	desired_position = Point()
	desired_position.x = goal.x
	desired_position.y = goal.y
	des_yaw = goal.theta

	success = True

	feedback = rt2_assignment1.msg.PositionFeedback()
	result = rt2_assignment1.msg.PositionResult()

	rate = rospy.Rate(20)
	
	change_state(0)
	while not rospy.is_shutdown():
		# if the goal is canceled
		if action_server.is_preempt_requested():
			action_server.set_preempted() # goal canceled
			success = False # goal not reached
			done() # stop the robot
			break
		
		elif state_ == 0:
			feedback.status = "Fixing the yaw angle"
			action_server.publish_feedback(feedback)
			fix_yaw(desired_position)

		elif state_ == 1:
			feedback.status = "Go straight, angle aligned"
			# publish the feedback status
			action_server.publish_feedback(feedback)
			go_straight_ahead(desired_position)
    		
		elif state_ == 2:
			feedback.status = "Fixing the final yaw angle"
			# publish the feedback status
			action_server.publish_feedback(feedback)
			fix_final_yaw(des_yaw)

		elif state_ == 3:
			feedback.status = "Target reached!"
			# publish the feedback status
			action_server.publish_feedback(feedback)
			done()
			break
		else:
			rospy.logerr('Unknown state!')

		rate.sleep()

	if success: # if the goal is reached
		rospy.loginfo('Goal: Succeeded!')
		result.success = True
		action_server.set_succeeded(result)

def main():
	##
	#	Main function that create a publisher on the topic cmd_vel,
	#	in order to publish new velocities, a subscriber on the
	#	topic odom, in order to read the actual position of the
	#	robot and start the action server.
	#
	
	global pub_, action_server
	rospy.init_node('go_to_point')
	pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	sub_consts = rospy.Subscriber('/consts', Twist, clbk_consts)
	action_server = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.PositionAction, go_to_point, auto_start=False)
	
	action_server.start()
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	main()
