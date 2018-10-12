Actionlib stuff

The code compiles, but the NUC is not working, so I can't try it.

The algorithm is basically a puse pursuit algorithm:

The robot is inside a circle, and you always look for the next point of the path that is outside the circle. It looks at the angle between the robot heading and the line that goes from the robot to the desired point, and sets an angular velocity that is proportional to that angle. The linear speed is set constant.

For this milestone I've created a "path_creator", as we don't have the map, and we don't have a defined path, this has been created as the simplest thing:a line. It has a function that returns a 2D array with the coordinates of different points on the line. The path should be taken from tha path planner in the future as a bunch of points in that format, so that the navigation algorithm can be reused.

There is also a pose cpp because I needed to translate the position I take from the odometry subscriber into a pose tipe message. This can be adapted into other types of messages if needed (when we have the localization we won't probably subscribe to the odometry any more).

The client subscribes to 2 topics:
	wall_detector: to cancel the action if it is too close to a wall
	destination: we send a destination point and it tells the server

The server subscribes to pose to know where the robot is and publishes velocities in the twist message so that the motors move.
