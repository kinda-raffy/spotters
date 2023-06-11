#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    Pose,
    Vector3,
    Point,
    Quaternion,
    PoseArray
)
from nav_msgs.msg import (
    OccupancyGrid,
    Path
)

# SETTINGS
# Map size
w = 5
h = 5


# Set the dummy map here
def map():
    map = OccupancyGrid()
    map.header.frame_id = "map"
    map.header.stamp = rospy.Time.now()

    map.info.width = w
    map.info.height = h
    map.info.resolution = 1

    for i in range(w * h):
        if i == w + 1:
            map.data.append(100)
        else:
            map.data.append(0)
        #map.data.append(0)

    return map

def curr_pos(x, y):
    curr_pos = PoseStamped()
    curr_pos.header.frame_id = "map"
    curr_pos.header.stamp = rospy.Time.now()

    curr_pos.pose.position.x = x
    curr_pos.pose.position.y = y
    curr_pos.pose.position.z = 0

    return curr_pos

def goal_pos(x, y):
    curr_pos = PoseStamped()
    curr_pos.header.frame_id = "map"
    curr_pos.header.stamp = rospy.Time.now()

    curr_pos.pose.position.x = x
    curr_pos.pose.position.y = y
    curr_pos.pose.position.z = 0

    return curr_pos


if __name__ == '__main__':
    print("Starting tester node")
    rospy.init_node('path_tester_node', anonymous=True)

    pub_map = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    pub_pos = rospy.Publisher('curr_pos', PoseStamped, queue_size=10)
    pub_goal = rospy.Publisher('goal_pos', PoseStamped, queue_size=10)

    rate = rospy.Rate(1)

    pos_x = 0
    pos_y = 0

    goal_x = 3
    goal_y = 3

    while not rospy.is_shutdown():
        pub_pos.publish(curr_pos(pos_x, pos_y))
        pub_goal.publish(goal_pos(goal_x, goal_y))
        pub_map.publish(map())

        rate.sleep()


        
        



