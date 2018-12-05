Documents of the path planning:

path_planner.cpp: Contains the A* algorithm, the functions to print the map and the path into two different txt files, function for thickening the walls of the given gridmap, function that converts the path into the correct format for the path follower, function for clearing start and ending points of the path, definition of the classs path.

class_cell.hpp: Contains the definition of the struct cell and the class set_of_cells, with all the functions that make easier to andle the data while path_planning.



The movement server gets a goal position from the client, as well as a starting position (also from the client). These two points are sent to the astar algorithm, in the path_planner.cpp file, and then the path follower commands the robot to move through the points created. The starting position is the robot's position, which is taken form the particle filter. The goal position is a point which can be an object that the robot has seen in the exploring phase or a point to explore (in the first phase).

The path following function is a pure-pursuit algorithm that commands the robot to move to the closest point of the path that is further than a thresshold (look-ahead distance). The velocitioes are set depending on the distance and on the heading of the robot and the point itself.
