# Path planner status
## Run In Spot?
- Remove tester.py from the launch file.
- Build and use the launch file as you would any ROS project.
- Change subscribers in path_planner.py as you wish

## TODO
- Pathfind out of a wall when stuck inside one.
- Test everything.

## Known Errors / Crashes
- In a small map, move in the same direction multiple times. Very weird crash, can't figure it out. Issue doesn't exist in bigger maps so should be no problem. Source: path_planner
```
path, g, rhs = dstar.move_and_replan(robot_position=new_position)
```
- If map is upside down, change these two lines in path_planner to y,x or x,y
```
navtask.set_goal_pos(navtask.cartesian_to_dstar((round(msg.pose.position.x), round(msg.pose.position.y))))
navtask.curr_pos = navtask.cartesian_to_dstar((round(msg.pose.position.x), round(msg.pose.position.y)))
```
- Right now, precision check is done for the map exact place in the grid. Might want to change this into something else (Path length < some amount) or similar.

## Credits
Original D* Pygame module: https://github.com/Sollimann/Dstar-lite-pathplanner
