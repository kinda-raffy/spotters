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
from navigator_new import OccupancyGridMap
from nav_msgs.msg import OccupancyGrid

from spot_driver.spot_ros import SpotROS

INITIAL_WIDTH = 1000
INITIAL_HEIGHT = 1000
current_pose = Pose() # Current position of the robot
next_pose = Pose() # Next pose within the path to go to.
# TODO: y_dim is apaprently the width. We need to confirm this through testing.
occupancy_grid_map = OccupancyGridMap(y_dim = INITIAL_WIDTH, x_dim = INITIAL_HEIGHT)

def occupancy_callback(data):
    # TODO - Update map
    y_dim = data.info.width
    x_dim = data.info.height
    occupancy_grid_map.set_map([data.data[i:i+y_dim] for i in range(0, len(data.data), y_dim)][:x_dim])
    
    # TODO - If goal exists, update path as well

def goal_callback(data):
    print("Received new goal")
    # TODO - Update path
    
def pose_callback(data):
    print("Received new pose")
    global current_pose
    current_pose = data
    # TODO - Update position in map based on current pose relative to start

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
    sub_pose = rospy.Subscriber('/pose', Pose, pose_callback)
    pub_path = rospy.Publisher('/spot_path', Pose, queue_size=10)

    rate = rospy.Rate(10)

    calc_path()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
