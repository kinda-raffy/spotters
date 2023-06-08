#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import (
    PoseStamped,
    PoseArray,
    Pose
)
start = (0, 0)
goal = (99, 99)

def curr_pose_publisher():
    curr_pose_pub = rospy.Publisher('curr_pos', Pose, queue_size=100)

    # Set the publishing rate (in Hz)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    curr_pos_msg = Pose()
    curr_pos_msg.position.x = start[0]
    curr_pos_msg.position.y = start[1]
    curr_pos_msg.position.z = 0

    while not rospy.is_shutdown():
        curr_pose_pub.publish(curr_pos_msg)

        # Sleep to maintain the publishing rate
        rate.sleep()

def goal_pose_publisher():
    goal_pose_pub = rospy.Publisher('goal_pos', Pose, queue_size=100)

    # Set the publishing rate (in Hz)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    goal_pos_msg = Pose()
    goal_pos_msg.position.x = goal[0]
    goal_pos_msg.position.y = goal[1]
    goal_pos_msg.position.z = 0

    while not rospy.is_shutdown():
        goal_pose_pub.publish(goal_pos_msg)

        # Sleep to maintain the publishing rate
        rate.sleep()
if __name__ == '__main__':
    try:
        rospy.init_node('start_goal_publisher_node')
        # curr_pose_publisher()
        goal_pose_publisher()
    except rospy.ROSInterruptException:
        pass
