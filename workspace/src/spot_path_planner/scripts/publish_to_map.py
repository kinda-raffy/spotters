#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

def publish_map():
    # Initialize ROS node and publisher
    rospy.init_node('map_publisher')
    map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

    # Create an OccupancyGrid message
    map_msg = OccupancyGrid()

    # Set necessary fields
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = 'map'
    map_msg.info.resolution = 0.1  # Adjust the resolution as needed
    map_msg.info.width = 100  # Adjust the map width as needed
    map_msg.info.height = 100  # Adjust the map height as needed
    map_msg.info.origin.position.x = 0.0  # Adjust the map origin as needed
    map_msg.info.origin.position.y = 0.0
    map_msg.info.origin.position.z = 0.0

    # Populate the map data
    map_msg.data = [0] * (map_msg.info.width * map_msg.info.height)

    # Publish the map message
    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        map_publisher.publish(map_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_map()
    except rospy.ROSInterruptException:
        pass
