import rospy
import time
from rt2_assignment1.srv import Command
import actionlib
import rt2_assignment1.msg

def main():
	rospy.init_node('user_interface')
	ui_client = rospy.ServiceProxy('/user_interface', Command)
	
	action_client = actionlib.SimpleActionClient("go_to_point",rt2_assignment1.msg.PositionAction)
	action_client.wait_for_server()
	
	time.sleep(10)
	rate = rospy.Rate(20)
	
	x = int(input("\nPress 1 to start the robot "))
	while not rospy.is_shutdown():
		if (x == 1):
			ui_client("start")
			x = int(input("\nPress 0 to stop the robot "))
		elif (x == 0):
			action_client.cancel_all_goals()
			ui_client("stop")
			print("Goal canceled")
			x = int(input("\nPress 1 to start the robot "))
		else:
			x = int(input("\nCommand not recognized, try again "))
			
			
if __name__ == '__main__':
    main()
