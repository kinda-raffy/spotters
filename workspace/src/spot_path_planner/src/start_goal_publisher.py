#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import (
    PoseStamped,
    PoseArray,
    Pose
)
start = (0, 0)
goal = (99, 99)

def pose_publisher():
    # Initialize the ROS node
    rospy.init_node('start_goal_publisher_node')
    nav_from_to_pub = rospy.Publisher('nav_from_to', PoseArray, queue_size=100)

    # Set the publishing rate (in Hz)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    pose_array = PoseArray()
    pose_array.header.frame_id = 'map'
    pose_array.header.stamp = rospy.Time.now()

    start_pos = Pose()
    start_pos.position.x = start[0]
    start_pos.position.y = start[1]
    start_pos.position.z = 0

    goal_pos = Pose()
    goal_pos.position.x = goal[0]
    goal_pos.position.y = goal[1]
    goal_pos.position.z = 0
    pose_array.poses.append(start_pos)
    pose_array.poses.append(goal_pos)

    while not rospy.is_shutdown():
        nav_from_to_pub.publish(pose_array)

        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
