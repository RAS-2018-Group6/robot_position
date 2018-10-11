#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include "math.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "path_creator.h"

double x_dest, y_dest;
bool obstacle =0;

void destinationCallback(const geometry_msgs::Pose::ConstPtr& msg){
	global x_dest = (double) msg -> position.x;
  global y_dest = (double) msg -> position.y;
}

void obstacleCallback(const std_msgs::Bool::ConstPtr& msg){
	global obstacle = (bool) msg -> data;

}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "send_movement");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_movement::MovementAction> ac("movement", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_movement::MovementGoal goal;
  
  ros::Subscriber dest = nh_.subscribe<geometry_msgs::Pose>("/destination", 10, &MovementAction::destinationCallback);
  ros::Subscriber obstacle = nh_.subscribe<std_msgs::Bool>("/obstacle", 10, &MovementAction::obstacleCallback);

  goal.final_point.position.x = x_dest;
  goal.final_point.position.y = y_dest;
  ac.sendGoal(goal);

  //wait for the action to return
  bool obstacle;

  if (obstacle)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

