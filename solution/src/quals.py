#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebase_client(goal_x, goal_y, orientation):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
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
        goal2 = [4.352, 6.547, 0.0, 0.0, 0.999, -0.034]
        rospy.init_node('solution', anonymous=True)
        rospy.loginfo(f"Setting Goal to ({goal1[0]}, {goal1[1]}), {goal1[2:]}")
        result1 = movebase_client(goal1[0], goal1[1], goal1[2:])
        if result1:
            rospy.loginfo("Goal 1 execution done!")
        rospy.loginfo(f"Setting Goal to ({goal2[0]}, {goal2[1]}), {goal2[2:]}")
        result2 = movebase_client(goal2[0], goal2[1], goal2[2:])
        if result2:
            rospy.loginfo("Goal 2 execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
