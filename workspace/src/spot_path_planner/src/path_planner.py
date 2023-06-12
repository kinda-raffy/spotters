#!/usr/bin/env python3
import rospy
from std_msgs.msg import (
    Header,
    Bool
)
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
from navtask import NavTask
import numpy as np

# from spot_driver.spot_ros import SpotROS

# As a prototype path planner, we assume that the robot has scouted the whole map before sending a goal_pos to navigate to.
# Therefore, goal_pos will be always within the map.

navtask = NavTask()

new_map = None
old_map = None
new_position = None
last_position = None
# TODO: Implement the translation from cartesian coordinates to d_star_lite indices
# curr_location = None
# goal_lcoation = None
dstar = None
slam = None
path = None

OBSTACLE = 100
UNOCCUPIED = 0

DEBUG = True

def map_callback(msg):
    global navtask
    
    navtask.map_width = msg.info.width
    navtask.map_height = msg.info.height 
    navtask.map_resolution = msg.info.resolution
    
    navtask.curr_map = np.reshape(msg.data, (navtask.map_height, navtask.map_width))

    # # Check the contents of map_data        
    # val_counts = {}
    # for num in map_data:
    #     if num in val_counts:
    #         val_counts[num] += 1
    #     else:
    #         val_counts[num] = 1
    # print(val_counts)

    # # Colour the entire map black and grey so that it can be visualised in rviz
    # map_pub = rospy.Publisher('map2', OccupancyGrid, queue_size = 100)
    # map_data = []
    # for idx, num in enumerate(msg.data):
    #     if idx < len(msg.data) // 2:
    #         map_data.append(50)
    #     else:
    #         map_data.append(100)
    # msg.data = map_data
    # map_pub.publish(msg)

def curr_pos_callback(msg):
    global navtask
    if navtask.map_width is not None:
        navtask.curr_pos = navtask.cartesian_to_dstar((round(msg.pose.position.y), round(msg.pose.position.x)))
    
    # Store the distance between the goal and the current postiions
    if navtask.is_set_up:
        navtask.curr_to_goal_distance = (navtask.goal_pos[0] - navtask.curr_pos[0],
                                          navtask.goal_pos[1] - navtask.curr_pos[1])
        

def goal_pos_callback(msg):
    global navtask
    if navtask.curr_pos is not None:
        navtask.set_goal_pos(navtask.cartesian_to_dstar((round(msg.pose.position.y), round(msg.pose.position.x))))

def is_localisation_lost_callback(msg):
    global navtask
    if msg.data == True:
        navtask.is_localisation_lost = True
    else:
        navtask.is_localisation_lost = False


if DEBUG:
    print("============================================")
    print("DEBUG MODE ON!")
    print("Source: path_planner from Spot Path Planner")
    print("============================================")

rospy.init_node('path_planner_node', anonymous=True)
pub_path = rospy.Publisher('path', Path, queue_size=10)
sub_map = rospy.Subscriber('map', OccupancyGrid, map_callback)
sub_curr_pos = rospy.Subscriber('curr_pos', PoseStamped, curr_pos_callback)
sub_goal_pos = rospy.Subscriber('goal_pos', PoseStamped, goal_pos_callback)
sub_is_localisation_lost = rospy.Subscriber('is_localisation_lost', Bool, is_localisation_lost_callback )

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    # If the localisation is lost, create a navtask again
    if navtask.is_localisation_lost:
        navtask = NavTask(navtask.curr_to_goal_distance)
        while navtask.curr_pos is None:
            rate.sleep()
        navtask.recover_goal_pos()
    while not navtask.is_ultimate_goal_reached() and not navtask.is_localisation_lost:
        # If the map isn't set up, then make the initial map.
        if navtask.is_set_up_needed():
            # y dim is the dimension in the direction of y; therefore it is equal to the width. 
            new_map = OccupancyGridMap(y_dim = navtask.map_width, x_dim = navtask.map_height)
            new_map.set_map(navtask.curr_map)
            new_position = navtask.curr_pos
            last_position = navtask.curr_pos

            dstar = DStarLite(map=new_map, s_start=navtask.curr_pos, s_goal=navtask.goal_pos)

            slam = SLAM(map=new_map, view_range=5)

            if navtask.is_out_of_bounds(new_position):
                    print("============================================")
                    print("WARNING! Might be out of bounds!")
                    navtask.is_set_up = False
            else:
                path, g, rhs = dstar.move_and_replan(robot_position=new_position)

                if DEBUG:
                    print("============================================")
                    print("Navtask setup success")
                    print("This includes map, current pos, and goal.")
                    print("Map width: " + str(navtask.map_width))
                    print("Map height: " + str(navtask.map_height))
                    print("Map resolution: " + str(navtask.map_resolution))
                    print("Current Position: " + str(navtask.curr_pos))
                    print("Target Position: " + str(navtask.goal_pos))

        # If the map is already setup, then do replanning when moving around.
        elif navtask.is_set_up:
            new_position = navtask.curr_pos
            new_map.set_map(navtask.curr_map)
            slam.set_ground_truth_map(gt_map=new_map)

            if new_position != last_position:
                last_position = new_position
                
                # slam
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position, curr_pos=new_position)

                dstar.new_edges_and_old_costs = new_edges_and_old_costs
                dstar.sensed_map = slam_map

                # d star
                if navtask.is_out_of_bounds(new_position):
                    print("============================================")
                    print("WARNING! Might be out of bounds!")
                else:
                    path, g, rhs = dstar.move_and_replan(robot_position=new_position)
                    if DEBUG:
                        print("============================================")
                        print("Replanning!")
        else:
            # The map isn't set up yet. Wait until it is.
            if DEBUG:
                print("============================================")
                print("Waiting for map!")
            rate.sleep()
            continue

        if len(path) > 1:
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = rospy.Time.now()

            for pos in path:
                pos_stamped = PoseStamped()
                pos_stamped.pose.position.x = pos[1]
                pos_stamped.pose.position.y = pos[0]
                # TODO: we assume that z position is 0 (this needs to be updated)
                pos_stamped.pose.position.z = 0
                path_msg.poses.append(pos_stamped)

            pub_path.publish(path_msg)
            if DEBUG:
                print("============================================")
                print("Published path!")
                print("Current Position: " + str(navtask.curr_pos))
                print("Target Position: " + str(navtask.goal_pos))
                pt = "Path: "
                for pos in path:
                    pt = pt + "[" + str(pos[1]) + " " + str(pos[0]) + "] "
                print(pt)
        elif len(path) == 1:
            if DEBUG:
                print("============================================")
                print("You have arrived - or is close to your destination.")
                print("Current Position: " + str(navtask.curr_pos))
                print("Target Position: " + str(navtask.goal_pos))
        
        rate.sleep()


    rate.sleep()
    
