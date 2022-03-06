#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from LocationDetector import LocationDetector
import cv2

def movebase_client(goal_x, goal_y, goal_ox, goal_oy, goal_oz, goal_ow):

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "husky_robot_model__base_link"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = goal_x
	goal.target_pose.pose.position.y = goal_y
	goal.target_pose.pose.orientation.x = goal_ox
	goal.target_pose.pose.orientation.y = goal_oy
	goal.target_pose.pose.orientation.z = goal_oz
	goal.target_pose.pose.orientation.w = goal_ow

	client.send_goal(goal)
	wait = client.wait_for_result()

	if not wait:
		rospy.logger("Action server not available")
		rospy.signal_shutdown("Action server not available")
	else:
		return client.get_result()

if __name__ == "__main__":
	# image = cv2.imread("../../images/image1.png")
	# lat, lon = LocationDetector().detect_location(image)
	# print(lat, lon)

	try:
		rospy.init_node('solution')
		result = movebase_client(8.1, -5.9, 0.0, 0.0, 0.7, 0.7)
		if result:
			rospy.loginfo("Goal execution done!")

	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")

