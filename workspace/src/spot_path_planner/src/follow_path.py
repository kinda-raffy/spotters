#!/usr/bin/env python3
import rospy
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
# TODO: from move_base_

# If True, then uses Rosbot movement commands. Otherwise, uses Spot setup.
USE_ROSBOT = True

from spot_driver.spot_ros import SpotROS

def path_callback(data):
    print("Path callback")

def pos_callback(data):
    print("Pos callback")

if __name__ == '__main__':
    if not USE_ROSBOT:
        SR = SpotROS()
        SR.main()

    rospy.init_node('follow_path_node', anonymous=True)
    
    sub_path = rospy.Subscriber('/path', Path, )
    sub_pos = rospy.Subscriber('/curr_pos', )

    if USE_ROSBOT:
        # TODO: Move base
        pub_drive = rospy.Publisher('/cmd_vel')
	

