# Files used in the final version 

## exploration_client.cpp
Movemenet client that serves as our brain. It combines all the different subsystems of the robot and then determines the robots beaviour depending on the different messages it receives from the sub systems. Depending on the selected mode (exploration or object retrieval) it determines different positions to go to and calls the movement server with this goal and additional behaviour information. Furthermore, by using different callback functions, different robot behaviors can be achieved, which decide on how to react in diffrent situations.

## movement_server.cpp
The movement server receives a goal position, the minimal distance to the goal (counts as reached), the movement direction (backwards, forwards), the velocity. In addition to that it can receive the distance to back off and which map to use for the path planning. Once it gets a goal from the brain the path planner is called and then the path is executed through a pure persuit controller until to robot is in a radius of "min_distance" to the goal.

## path_planner.cpp
Path planner used. For more information see: README_Path_planner.md

# Additional files

## abortDueToWall.cpp
Code that sends a message to the brain to cancel the goal/current path execution, because it detected a wall to close to the robot with the Lidar.

## backwards_server.cpp
Additional server to specifically command the robot to drive backwards.

## movement_client.cpp
First movement_client. Was replaced by the exploration_client.

## object_client.cpp
Brain that contains the first initial behaviour strategies to retrieve an object once an object was detected by the camera.

## create_path.cpp
Initial path planner(very simple)
