# Path planner status
## Run
- Remove tester.py from the launch file.
- Build and use the launch file as you would any ROS project.
- Change subscribers in path_planner.py as you wish

## Known Errors / Crashes
- In a small map, move in the same direction multiple times. Very weird crash, can't figure it out. Issue doesn't exist in bigger maps so should be no problem. Source: Around Line 173 - Path Planner

## Credits
Original D* Pygame module: https://github.com/Sollimann/Dstar-lite-pathplanner
