# Spot Path Planner
This project should:
* Be provided the available occupancy grid, which for now is expected to be in terms of (X, Y)
* Be able to plan Spot's path to a given target point
* Calculate target points to move towards within the path
* Publish the next target position for Spot to move towards
* Recalculate parts of the path if necessary

Moving Spot itself should probably not be hosted inside this project; another project would be the best solution.