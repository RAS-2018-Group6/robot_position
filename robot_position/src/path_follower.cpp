#include <ros/ros.h>
#include "path_creator.h"
#include "math.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

int follow(double **path, i){
	//Transform points into base frame
	//See if (path [i] is outside of the circle)
	//If it is outside, calculate angle and set velocity
	//If it is inside, i++ and check again step 2 
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
