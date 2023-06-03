#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(map_msg):
    # Handle the received map data here
    # This function will be called whenever a new map message is received
    rospy.loginfo(map_msg)

def subscribe_to_map():
    # Initialize ROS node and subscriber
    rospy.init_node('map_subscriber')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Spin ROS
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_to_map()
    except rospy.ROSInterruptException:
        pass