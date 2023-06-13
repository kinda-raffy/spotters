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

# =========================================================================
# =                             SETTINGS                                  =
NODE_ID = "path_planner_node"
MAP_SUBSCRIBER_TOPIC = "spotters/mapping/map"
POS_SUBSCRIBER_TOPIC = "spotters/mapping/pos"
GOAL_SUBSCRIBER_TOPIC = "spotters/conductor/goal"
LOST_SUBSCRIBER_TOPIC = "spotters/cartographer/tracking_state"
PATH_PUBLISHER_TOPIC = "spotters/navigator/path"
# =                                                                       =
# =========================================================================

navtask = NavTask()

new_map = None
old_map = None
new_position = None
last_position = None
last_goal = None
dstar = None
slam = None
path = None

posesraw = []

OBSTACLE = 100
UNOCCUPIED = 0

DEBUG = True

def map_callback(msg):
    global navtask
    
    navtask.map_width = msg.info.width
    navtask.map_height = msg.info.height 
    navtask.map_resolution = msg.info.resolution
    
    navtask.map_offset_x = msg.info.origin.position.x
    navtask.map_offset_y = msg.info.origin.position.y
    
    #real_map_width = msg.info.width * msg.info.resolution

    navtask.curr_map = np.reshape(msg.data, (navtask.map_height, navtask.map_width))

def curr_pos_callback(msg):
    global navtask
    if navtask.map_width is not None:
        
        # TEMP Debug
        # print("Raw: " + str(msg.pose.position.y) + " / " + str(msg.pose.position.x))
        # posesraw.append([msg.pose.position.x, msg.pose.position.y])
        # t = ""
        # for pos in posesraw:
        #     t += "[" + str(pos[0]) + " " + str(pos[1]) +"] "
        # print(posesraw)
        #print("Orientation: " + str(-msg.pose.orientation.x) + " / " + str(-msg.pose.orientation.y) + " / " + str(-msg.pose.orientation.z))
        #print("Cartesian: " + str(-msg.pose.position.x - navtask.map_offset_x) + " / " + str(-msg.pose.position.y) + " / " + str(-msg.pose.position.z))
        
        navtask.curr_pos = (round(msg.pose.position.y * 1 / navtask.map_resolution - navtask.map_offset_y), round(msg.pose.position.x * 1 / navtask.map_resolution - navtask.map_offset_x))
    
    # Store the distance between the goal and the current postiions
    if navtask.is_set_up:
        navtask.curr_to_goal_distance = (navtask.goal_pos[0] - navtask.curr_pos[0],
                                          navtask.goal_pos[1] - navtask.curr_pos[1])
        
def goal_pos_callback(msg):
    global navtask
    if navtask.curr_pos is not None:
        navtask.set_goal_pos((round(msg.pose.position.y * 1 / navtask.map_resolution - navtask.map_offset_y), round(msg.pose.position.x * 1 / navtask.map_resolution - navtask.map_offset_x)))

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

rospy.init_node(NODE_ID, anonymous=True)
pub_path = rospy.Publisher(PATH_PUBLISHER_TOPIC, Path, queue_size=10)
pub_goal = rospy.Publisher(GOAL_SUBSCRIBER_TOPIC, PoseStamped, queue_size=10)

sub_map = rospy.Subscriber(MAP_SUBSCRIBER_TOPIC, OccupancyGrid, map_callback)
sub_curr_pos = rospy.Subscriber(POS_SUBSCRIBER_TOPIC, PoseStamped, curr_pos_callback)
sub_goal_pos = rospy.Subscriber(GOAL_SUBSCRIBER_TOPIC, PoseStamped, goal_pos_callback)
sub_is_localisation_lost = rospy.Subscriber(LOST_SUBSCRIBER_TOPIC, Bool, is_localisation_lost_callback)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    
    # TEMP
    # ===
    gp = PoseStamped()
    gp.pose.position.x = 0
    gp.pose.position.y = 0
    pub_goal.publish(gp)
    # ===
    
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
            last_goal = navtask.goal_pos

            dstar = DStarLite(map=new_map, s_start=navtask.curr_pos, s_goal=navtask.goal_pos)

            slam = SLAM(map=new_map, view_range=5)

            if navtask.is_out_of_bounds(new_position):
                    print("============================================")
                    print("WARNING! Might be out of bounds!")
                    print("Val: " + str(navtask.curr_pos))
                    navtask.is_set_up = False
            else:
                path, g, rhs = dstar.move_and_replan(robot_position=new_position)
                if path is None:
                    navtask.is_set_up = False
                    continue

                if DEBUG:
                    print("============================================")
                    print("Navtask setup success")
                    print("This includes map, current pos, and goal.")
                    print("Map width: " + str(navtask.map_width))
                    print("Map height: " + str(navtask.map_height))
                    print("Map resolution: " + str(navtask.map_resolution))
                    print("Map offset x: " + str(navtask.map_offset_x))
                    print("Map offset y: " + str(navtask.map_offset_y))
                    
                    # x, y = navtask.curr_pos
                    # x, y = (round(x * navtask.map_resolution, 2), round(y * navtask.map_resolution, 2))
                    # print("Current Position in cartesian: " + "[" + str(x) + " " + str(y) + "] ")
                    
                    # print("Current Position in dstar: " + str(navtask.curr_pos))
                    
                    # x, y = navtask.goal_pos
                    # x, y = (round(x * navtask.map_resolution, 2), round(y * navtask.map_resolution, 2))
                    # print("Target Position in cartesian: " + "[" + str(x) + " " + str(y) + "] ")
                    
                    # print("Target Position in dstar: " + str(navtask.goal_pos))

        # If the map is already setup, then do replanning when moving around.
        elif navtask.is_set_up:
            if navtask.goal_pos != last_goal:
                if DEBUG:
                    print("============================================")
                    print("Got new goal: " + str(navtask.goal_pos))
                    print("Resetting navtask...")
                navtask.is_set_up = False

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
                    print("Val: " + str(new_position))
                else:
                    path, g, rhs = dstar.move_and_replan(robot_position=new_position)
                    if path is None:
                        navtask.is_set_up = False
                        continue
                    
                    if DEBUG:
                        print("============================================")
                        print("Replanning!")
        else:
            # The data isn't set up yet. Wait until it is.
            if DEBUG:
                print("============================================")
                print("Waiting for additional data! (Missing either goal, current pos, or map)")
            rate.sleep()
            continue

        if path is None:
            if navtask.is_out_of_bounds(new_position):
                print("============================================")
                print("WARNING: Failed to calculate path!")
        elif len(path) > 1:
            path_msg = Path()
            path_msg.header.frame_id = 'origin'
            path_msg.header.stamp = rospy.Time.now()

            for pos in path:
                pos_stamped = PoseStamped()
                x, y = (pos[0] * navtask.map_resolution + navtask.map_offset_x, pos[1] * navtask.map_resolution + navtask.map_offset_y)
                pos_stamped.pose.position.x = x
                pos_stamped.pose.position.y = y
                pos_stamped.pose.position.z = 0
                path_msg.poses.append(pos_stamped)

            pub_path.publish(path_msg)
            if DEBUG:
                print("============================================")
                print("Published path!")
                
                # TEMP Debug
                # x, y = navtask.curr_pos
                # x, y = (round(x * navtask.map_resolution, 2), round(y * navtask.map_resolution, 2))
                # print("Current Position in cartesian: " + "[" + str(x) + " " + str(y) + "] ")
                
                #print("Current Position in dstar: " + str(navtask.curr_pos))
                
                # x, y = navtask.goal_pos
                # x, y = (round(x * navtask.map_resolution, 2), round(y * navtask.map_resolution, 2))
                # print("Target Position in cartesian: " + "[" + str(x) + " " + str(y) + "] ")
                
                # print("Target Position in dstar: " + str(navtask.goal_pos))
                
                pt = "Path in cartesian: "
                for pos in path:
                    x, y = (pos[0], pos[1])
                    x, y = (round(x * navtask.map_resolution + navtask.map_offset_x, 2), round(y * navtask.map_resolution + navtask.map_offset_y, 2))
                    pt = pt + "[" + str(x) + " " + str(y) + "] "
                    
                print(pt)
        elif len(path) == 1:
            if DEBUG:
                print("============================================")
                print("You have arrived - or is close to your destination.")
                print("Current Position: " + str(navtask.curr_pos))
                print("Target Position: " + str(navtask.goal_pos))
        
        rate.sleep()


    rate.sleep()
    
