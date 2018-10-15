#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_movement/MovementAction.h>
#include "math.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>

#include <std_msgs/Bool.h>

double x_dest, y_dest;
bool obstacle_on = 0;
actionlib_movement::MovementGoal goal;

void destinationCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
   x_dest = (double) msg->point.x;
   y_dest = (double) msg->point.y;

   ROS_INFO("X: %f , Y:%f",x_dest,y_dest);

}

void obstacleCallback(const std_msgs::Bool::ConstPtr& msg){
   obstacle_on = (bool) msg -> data;

}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "send_movement");
  ros::NodeHandle nh_;
  bool obstacle_on;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_movement::MovementAction> ac("movement", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal. GO BANANAS!");
  // send a goal to the action


  ros::Subscriber dest = nh_.subscribe<geometry_msgs::PointStamped>("/found_object", 1, destinationCallback);
  ros::Subscriber obstacle = nh_.subscribe<std_msgs::Bool>("/wall_detected", 1, obstacleCallback);


  //while (ros::ok())
  //{
    //ros::spinOnce();
    //x_dest = 10;
    //y_dest = 10;
    goal.final_point.position.x = x_dest;// x_dest;
    goal.final_point.position.y = y_dest; //y_dest;
    ac.sendGoal(goal);

    //wait for the action to return


    if (obstacle_on)
    {

      	//The goal should be cancelled here
      	ac.cancelGoal();
      	//ROS_INFO  ("The goal has been cancelled");
        //actionlib::SimpleClientGoalState state = ac.getState();
        //ROS_INFO("Action finished: %s",state.toString().c_str());
      }
      else{
        //ROS_INFO("Action did not finish before the time out.");
      }

      //ac.waitForResult();
      ros::spin();

      //exit
  //}
  return 0;
}
