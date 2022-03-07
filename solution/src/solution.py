#!/usr/bin/env python3
import cv2
import os
import numpy as np

import rospy
import actionlib
from coordi_msgs.msg import coordi
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from LocationDetector import LocationDetector

image = None
orientation = None


def callback1(data):
    global image, orientation
    image = data.image
    orientation = data.orientation


def movebase_client(goal_x, goal_y, orientation):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "husky_robot_model__base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logger("Action server not available")
        rospy.signal_shutdown("Action server not available")
    else:
        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.Subscriber('/terrorist_location', coordi, callback1)
        rospy.init_node('solution', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            if image and orientation:
                orientation = list(orientation)
                image = np.array(image)
                image = image.reshape([379, 1049, 3])
                image = image.astype(np.uint8)
                lat, lon = LocationDetector().detect_location(image)
                rospy.loginfo(f"New Goal Position : {lat}, {lon}, {orientation}")
                result = movebase_client(lat, lon, orientation)
                if result:
                    rospy.loginfo("Goal execution done!")
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
