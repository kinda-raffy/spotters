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
from nav_msgs.msg import OccupancyGrid


from navigator.d_star_lite import DStarLite
from navigator.grid import OccupancyGridMap, SLAM


from spot_driver.spot_ros import SpotROS

# As a prototype path planner, we assume that the robot has scouted the whole map before sending a goal_pos to navigate to.
# Therefore, goal_pos will be always within the map.

start_pos = None
goal_pos = None
initial_width = None
initial_height = None
new_map = None
old_map = None
new_position = None
last_position = None
dstar = None
slam = None
is_goal_reached = False

def publish_path(pub_path, message, rate):
    while not rospy.is_shutdown():
        pub_path.publish(message)
        rate.sleep()
    

def main():
    SR = SpotROS()
    SR.main()

    rospy.init_node('path_planner', anonymous=True)

    # Assume that the start and goal positions in the map frame are published to /nav_from_to
    nav_msg = rospy.wait_for_message('nav_from_to', PoseArray)
    # Set start and goal positions to discretised values
    start_pos = (round(nav_msg.poses[0].position.x), round(nav_msg.poses[0].position.y))
    goal_pos =  (round(nav_msg.poses[1].position.x), round(nav_msg.poses[1].position.y))

    while not rospy.is_shutdown() and not is_goal_reached:
        map_msg = rospy.wait_for_message('map', OccupancyGrid) 
        if new_map is None:
            initial_width = map_msg.info.width
            initial_height = map_msg.info.height
            # y dim is the dimension in the direction of y; therefore it is equal to the width. 
            new_map = OccupancyGridMap(y_dim = initial_width, x_dim = initial_height)
            new_map.set_map([map_msg.data[i:i+initial_width] for i in range(0, len(map_msg.data), initial_width)][:initial_height])
            old_map = new_map

            new_position = start_pos
            last_position = start_pos

            dstar = DStarLite(map=new_map, s_start=start_pos, s_goal=goal_pos)

            slam = SLAM(map=new_map, view_range=5)
            
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
        else:
            # Assume that the robot's position in the map frame is published to curr_pos
            curr_pos_msg = rospy.wait_for_message('curr_pos', Pose)
            new_position = (round(curr_pos_msg.position.x), round(curr_pos_msg.position.y))
            new_map.set_map()
            new_map.set_map([map_msg.data[i:i+initial_width] for i in range(0, len(map_msg.data), initial_width)][:initial_height])
            old_map = new_map
            slam.set_ground_truth_map(gt_map=new_map)

            if new_position != last_position:
                last_position = new_position
                
                # slam
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

                dstar.new_edges_and_old_costs = new_edges_and_old_costs
                dstar.sensed_map = slam_map

                # d star
                path, g, rhs = dstar.move_and_replan(robot_position=new_position)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
