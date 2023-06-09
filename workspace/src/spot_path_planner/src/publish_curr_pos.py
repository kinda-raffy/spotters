#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

ROBOT_NAME = 'targaryen/'

def odom_callback(msg):
    global tf_buffer, curr_pos_pub

    # Convert the odometry message to a pose stamped message
    odom_pose = PoseStamped()
    odom_pose.header = msg.header
    odom_pose.pose = msg.pose.pose

    # Lookup the transform from the odom frame to the map frame
    try:
        transform = tf_buffer.lookup_transform("map", "odom", rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to lookup transform: %s" % str(e))
        return

    # Transform the pose from the odom frame to the map frame
    try:
        map_pose = tf2_geometry_msgs.do_transform_pose(odom_pose, transform)
    except tf2_ros.TransformException as e:
        rospy.logwarn("Failed to transform pose: %s" % str(e))
        return

    # Append the transformed pose to the path
    curr_pos_pub.pose = map_pose.pose

if __name__ == '__main__':
    rospy.init_node('path_publisher')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    curr_pos_pub = PoseStamped()
    curr_pos_pub.header.frame_id = "map"

    rospy.Subscriber(ROBOT_NAME + "odom", Odometry, odom_callback)
    pub = rospy.Publisher('/curr_pos', PoseStamped, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        curr_pos_pub.header.stamp = rospy.Time.now()
        pub.publish(curr_pos_pub)
        rate.sleep()

    rospy.spin()