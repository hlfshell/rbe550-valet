# RBE550 Valet Project

This is the RBE550 Valet homework assignment, completed for the Spring 2022 term.

The `doc` section contains the assignment writeup. The `videos` folder contains examples of each of these robots working.

The following scripts are your entrypoints:

* `skid.py` - Navigate a map with the skid drive robot
* `ackermann.py` - Navigate a map with the Ackermann drive robot
* `trailer.py` - Navigate a map with the Ackermann drive robot and an attached trailer
* `draw_map.py` - Draw obstacles for a map. Click to place a point, right click to complete the obstacle (if it has at least three points). Right click with no obstacles to save the map as `tmp.map`.

Each robot script will wait for you to place the initial pose of the robot. Every pose placement there after will be your new goal pose.

To change maps, change the loaded map in the associated script for your chosen robot.