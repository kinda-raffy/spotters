import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    Pose,
    Vector3,
    Point,
    Quaternion
)

from navigator import grid, graph, d_star_lite
from nav_msgs.msg import OccupancyGrid

from spot_driver.spot_ros import SpotROS

def occupancy_callback(data):
    print("Received occupancy grid")

def goal_callback(data):
    print("Received new goal")

def publish_path(pub_path, message, rate):
    while not rospy.is_shutdown():
        pub_path.publish(message)
        rate.sleep()

# Should return the path needed to publish
def calc_path():
    print("Calculating Path")
    

def main():
    SR = SpotROS()
    SR.main()

    rospy.init_node('path_planner', anonymous=True)
    
    sub_goal = rospy.Subscriber('/goal', Pose, occupancy_callback)
    sub_grid = rospy.Subscriber('/grid', OccupancyGrid, occupancy_callback)
    pub_path = rospy.Publisher('/spot_path', Pose, queue_size=10)

    rate = rospy.Rate(10)

    calc_path()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
