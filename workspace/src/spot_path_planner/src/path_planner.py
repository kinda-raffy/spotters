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



from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM


# from spot_driver.spot_ros import SpotROS

# As a prototype path planner, we assume that the robot has scouted the whole map before sending a goal_pos to navigate to.
# Therefore, goal_pos will be always within the map.

curr_pos = None
start_pos = None
goal_pos = None
map_width = None
map_height = None
map_resolution = None
new_map = None
curr_map = None
old_map = None
new_position = None
last_position = None
dstar = None
slam = None
is_goal_reached = False
path = None
is_planning_started = False
curr_pos_z = None

ROBOT_NAME = "/"

from collections import Counter

def map_callback(msg):
    global map_width, map_height, map_resolution, curr_map, curr_pos, start_pos

    map_width = msg.info.width
    map_height = msg.info.height
    map_resolution = msg.info.resolution

    val_counts = {}

    map_data = []
    for num in msg.data:
        if num != -1:
            map_data.append(num)
        else:
            map_data.append(0)

    for num in map_data:
        if num in val_counts:
            val_counts[num] += 1
        else:
            val_counts[num] = 1

    print(val_counts)
    
    curr_map = [map_data[i:i+map_width] for i in range(0, len(map_data), map_width)][:map_height]
            

    # After the map is updated, read the curr_pos in the updated map frame
    curr_pos_msg = rospy.wait_for_message('curr_pos', PoseStamped)
    curr_pos = (round(curr_pos_msg.pose.position.x / map_resolution) + map_height // 2, round(curr_pos_msg.pose.position.y / map_resolution) + map_width // 2)
    
    if start_pos is None:
        start_pos = curr_pos


if __name__ == '__main__':
    rospy.init_node('path_planner_node', anonymous=True)
    pub_path = rospy.Publisher('path', Path, queue_size=10)
    sub_map = rospy.Subscriber(ROBOT_NAME + 'map', OccupancyGrid, map_callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown() and not is_goal_reached:
        if map_resolution is not None:
            print("GO AHEAD")
            goal_pos_msg = rospy.wait_for_message('goal_pos', PoseStamped)
            print(goal_pos_msg.pose)
            print(map_resolution)
            goal_pos  = ((round(goal_pos_msg.pose.position.x / map_resolution)) + map_height // 2, (round(goal_pos_msg.pose.position.y / map_resolution)) + map_width // 2)
            
            if not is_planning_started:
                # y dim is the dimension in the direction of y; therefore it is equal to the width. 
                new_map = OccupancyGridMap(y_dim = map_width, x_dim = map_height)
                new_map.set_map(curr_map)

                new_position = start_pos
                last_position = start_pos

                dstar = DStarLite(map=new_map, s_start=start_pos, s_goal=goal_pos)

                slam = SLAM(map=new_map, view_range=5)
                
                path, g, rhs = dstar.move_and_replan(robot_position=new_position)

                is_planning_started = True
            else:
                new_position = curr_pos
                new_map.set_map(curr_map)
                slam.set_ground_truth_map(gt_map=new_map)

                if new_position != last_position:
                    last_position = new_position
                    
                    # slam
                    new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

                    dstar.new_edges_and_old_costs = new_edges_and_old_costs
                    dstar.sensed_map = slam_map

                    # d star
                    path, g, rhs = dstar.move_and_replan(robot_position=new_position)

            # If the length of path is 1, set is_goal_reached to true
            if len(path) == 1:
                is_goal_reached = True
        
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = rospy.Time.now()

            for pos in path:
                pos_stamped = PoseStamped()
                pos_stamped.pose.position.x = (pos[0] - map_height / 2) * map_resolution
                pos_stamped.pose.position.y = (pos[1] - map_width / 2) * map_resolution
                # TODO: fix z position
                pos_stamped.pose.position.z = 0
                path_msg.poses.append(pos_stamped)
            pub_path.publish(path_msg)

        rate.sleep()
