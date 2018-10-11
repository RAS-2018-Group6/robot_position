#include <ros/ros.h>
#include "path_creator.h"
#include "math.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

int follow(double **path, i){
	//See if (path [i] is outside of the circle)
	int radius =0.06; //meters
	while (sqrt(pow(path[i][0]-x_robot,2)+pow(path[i][1]-y_robot,2))<= radius) i++;
	
	//Calculate angle and publish velocity
	double phi = atan2 ((path[i][1]-y_robot)/(path[i][0]-x_robot));
	//publish angular_velocity = k * phi;
	 
	if (!points) {break;}
	return i;
}
int main (int argc, char **argv)
{
  path path_points;
  int i = 0;
  while (true){
  	i = follow (path_points, i)
  	i++
  }
  
  return 0;
}
