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

# =========================================================================
# =                             SETTINGS                                  =
NODE_ID = "path_tester_node"
MAP_PUBLISHER_TOPIC = "spotters/geography/map"
POS_PUBLISHER_TOPIC = "spotters/geography/pos"
GOAL_PUBLISHER_TOPIC = "spotters/conductor/goal"
LOST_PUBLISHER_TOPIC = "spotters/cartographer/tracking_state"
PATH_SUBSCRIBER_TOPIC = "spotters/navigator/path"
# =                                                                       =
# =========================================================================

# SETTINGS FOR TESTS
# Map size
w = 150
h = 150

pos_x = 0
pos_y = 0

goal_x = 30
goal_y = -30
# END

# Set the dummy map here
def map():
    map = OccupancyGrid()
    map.header.frame_id = "origin"
    map.header.stamp = rospy.Time.now()

    map.info.width = w
    map.info.height = h
    map.info.resolution = 1

    for i in range(w * h):
        if (i > w * 86 - 1 and i < w * 89 - 1) or (i > w * 28 and i < w * 31):
            map.data.append(100)
        else:
            map.data.append(0)
        # map.data.append(0)

    return map

def curr_pos(x, y):
    curr_pos = PoseStamped()
    curr_pos.header.frame_id = "origin"
    curr_pos.header.stamp = rospy.Time.now()

    curr_pos.pose.position.x = x
    curr_pos.pose.position.y = y
    curr_pos.pose.position.z = 0

    return curr_pos

def goal_pos(x, y):
    curr_pos = PoseStamped()
    curr_pos.header.frame_id = "origin"
    curr_pos.header.stamp = rospy.Time.now()

    curr_pos.pose.position.x = x
    curr_pos.pose.position.y = y
    curr_pos.pose.position.z = 0

    return curr_pos

def path_next(data):
    global pos_x, pos_y
    
    #nextpose = data.poses[1]
    #pos_x = round(nextpose.pose.position.x)
    #pos_y = round(nextpose.pose.position.y)


if __name__ == '__main__':
    print("Starting tester node")
    rospy.init_node(NODE_ID, anonymous=True)

    pub_map = rospy.Publisher(MAP_PUBLISHER_TOPIC, OccupancyGrid, queue_size=10)
    pub_pos = rospy.Publisher(POS_PUBLISHER_TOPIC, PoseStamped, queue_size=10)
    pub_goal = rospy.Publisher(GOAL_PUBLISHER_TOPIC, PoseStamped, queue_size=10)
    sub_path = rospy.Subscriber(PATH_SUBSCRIBER_TOPIC, Path, path_next)

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        pub_pos.publish(curr_pos(pos_x, pos_y))
        pub_goal.publish(goal_pos(goal_x, goal_y))
        pub_map.publish(map())

        #rate.sleep()
        s = input()
        if s == 'w':
            pos_y += 1
        elif s == 'd':
            pos_x += 1
        elif s == 's':
            pos_y -= 1
        elif s == 'a':
            pos_x -= 1


        
        



