#!/usr/bin/env python3
import rospy
from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM
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

OBSTACLE = 100
UNOCCUPIED = 0

if __name__ == '__main__':
    rospy.init_node('gui_main_node', anonymous=True)

    """
    set initial values for the map occupancy grid
    |----------> y, column
    |           (x=0,y=2)
    |
    V (x=2, y=0)
    x, row
    """
    x_dim = 100
    y_dim = 100
    start = (0, 0)
    goal = (99,99)
    view_range = 5

    gui = Animation(title="D* Lite Path Planning",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    viewing_range=view_range)

    new_map = gui.world
    old_map = new_map

    new_position = start
    last_position = start

    # new_observation = None
    # type = OBSTACLE

    # D* Lite (optimized)
    dstar = DStarLite(map=new_map,
                      s_start=start,
                      s_goal=goal)

    # SLAM to detect vertices
    slam = SLAM(map=new_map,
                view_range=view_range)

    # move and compute path
    path, g, rhs = dstar.move_and_replan(robot_position=new_position)

    map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=100)
    curr_pos_pub = rospy.Publisher('curr_pos', Pose, queue_size=100)

    rate = rospy.Rate(1)

    while not gui.done:
        # update the map
        # print(path)
        # drive gui
        gui.run_game(path=path)

        new_map = gui.world
        print(new_map.occupancy_grid_map)

        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = 1
        map_msg.info.width = y_dim
        map_msg.info.height = x_dim
        map_msg.data = [cell for row in new_map.occupancy_grid_map for cell in row]
        
        map_pub.publish(map_msg)


        new_position = gui.current

        curr_pos_msg = Pose()
        curr_pos_msg.position.x = new_position[0]
        curr_pos_msg.position.y = new_position[1]
        curr_pos_msg.position.z = 0
        curr_pos_pub.publish(curr_pos_msg)

        

        """
        if new_observation is not None:
            if new_observation["type"] == OBSTACLE:
                dstar.global_map.set_obstacle(pos=new_observation["pos"])
            if new_observation["pos"] == UNOCCUPIED:
                dstar.global_map.remove_obstacle(pos=new_observation["pos"])
        """

       
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
        
        rate.sleep()