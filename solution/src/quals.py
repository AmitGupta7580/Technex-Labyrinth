#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


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
        goal1 = [-4.74, -1.68, 0.0, 0.0, -0.7076, 0.7065]
        rospy.init_node('solution', anonymous=True)
        rospy.loginfo(f"Setting Goal to ({goal1[0]}, {goal1[1]}), {goal1[2:]}")
        result = movebase_client(goal1[0], goal1[1], goal1[2:])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
