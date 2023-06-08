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




def publish_path(pub_path, message, rate):
    while not rospy.is_shutdown():
        pub_path.publish(message)
        rate.sleep()
    

def main():
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
    path = None
    # SR = SpotROS()
    # SR.main()
    rospy.init_node('path_planner', anonymous=True)
    pub_path = rospy.Publisher('path', Path, queue_size=10)

    while not rospy.is_shutdown() and not is_goal_reached:
        map_msg = rospy.wait_for_message('map', OccupancyGrid) 
        curr_pos_msg = rospy.wait_for_message('curr_pos', Pose)
        # If it is the first time this code block is being executed,
        if start_pos is None:
            start_pos = (round(curr_pos_msg.position.x), round(curr_pos_msg.position.y))
        goal_pos_msg = rospy.wait_for_message('goal_pos', Pose)
        goal_pos  = (round(goal_pos_msg.position.x), round(goal_pos_msg.position.y))
        
        if new_map is None:
            initial_width = map_msg.info.width
            initial_height = map_msg.info.height
            # y dim is the dimension in the direction of y; therefore it is equal to the width. 
            new_map = OccupancyGridMap(y_dim = initial_width, x_dim = initial_height)
            new_map.set_map([map_msg.data[i:i+initial_width] for i in range(0, len(map_msg.data), initial_width)][:initial_height])

            new_position = start_pos
            last_position = start_pos

            dstar = DStarLite(map=new_map, s_start=start_pos, s_goal=goal_pos)

            slam = SLAM(map=new_map, view_range=5)
            
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
        else:
            new_position = (round(curr_pos_msg.position.x), round(curr_pos_msg.position.y))
            new_map.set_map([map_msg.data[i:i+initial_width] for i in range(0, len(map_msg.data), initial_width)][:initial_height])
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
            pos_stamped.pose.position.x = pos[1]
            pos_stamped.pose.position.y = pos[0]
            pos_stamped.pose.position.z = curr_pos_msg.position.z
            path_msg.poses.append(pos_stamped)
        pub_path.publish(path_msg)

       

        
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
