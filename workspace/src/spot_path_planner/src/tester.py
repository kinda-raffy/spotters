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
w = 10
h = 10

# Global vars
pos_x = 0
pos_y = 0

goal_x = 9
goal_y = 9


# Set the dummy map here
def map():
    map = OccupancyGrid()
    map.header.frame_id = "map"
    map.header.stamp = rospy.Time.now()

    map.info.width = w
    map.info.height = h
    map.info.resolution = 1

    for i in range(w * h):
        if i > w * 4 - 1 and i < w * 5 - 1:
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

def path_next(data):
    global pos_x, pos_y

    nextpose = data.poses[1]
    pos_x = round(nextpose.pose.position.x)
    pos_y = round(nextpose.pose.position.y)

if __name__ == '__main__':
    print("Starting tester node")
    rospy.init_node('path_tester_node', anonymous=True)

    pub_map = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    pub_pos = rospy.Publisher('curr_pos', PoseStamped, queue_size=10)
    pub_goal = rospy.Publisher('goal_pos', PoseStamped, queue_size=10)
    sub_path = rospy.Subscriber('path', Path, path_next)

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        pub_pos.publish(curr_pos(pos_x, pos_y))
        pub_goal.publish(goal_pos(goal_x, goal_y))
        pub_map.publish(map())

        rate.sleep()


        
        



