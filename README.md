autonomous_charging
===================

Contains some functionalities to allow the robot to detect the docking station and drive to it

You can run the ROS node that gets the laser scans as follows:
rosrun detect_docking_station docking_station_detector _laser:=/scan

Make sure you have the simulation environment running first.

ISSUES
===================
Obsticle avoidance for path-planning in the nav-stack is stopping us from getting closer to the charger.
This can be fixed by changing a parameter(obstacle_range in costmap_common_params.yaml) in the navstack or making a new planner/controller.

