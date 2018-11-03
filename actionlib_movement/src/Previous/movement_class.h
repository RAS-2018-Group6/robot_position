#ifndef movement_class.h
#define movement_class.h
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> //Action library used from implementing simple actions
#include <actionlib_movement/MovementAction.h>
#include "math.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "path_creator.h"


class MovementAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_movement::MovementAction> as_; // Action Server
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_movement::MovementFeedback feedback_;
  actionlib_movement::MovementResult result_;
  double x_robot, y_robot, phi_robot;
	path mypath;
	geometry_msgs::Twist vel;
	float a = 0.2;
	float k = 30;

	
public:

	
	//Action constructor

  MovementAction(std::string name) :
    as_(nh_, name, boost::bind(&MovementAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    //ros::Subscriber sub_pose = nh_.subscribe<geometry_msgs::Pose>("/pose", 10, &MovementAction::poseCallback, this);
    //ros::Publisher pub_vel = nh_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
		
  }
  
  ros::Publisher pub_vel = nh_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
  ros::Subscriber sub_pose = nh_.subscribe<geometry_msgs::Pose>("/pose", 10, &MovementAction::poseCallback, this);
  
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
  	x_robot = (double) msg -> position.x;
  	y_robot = (double) msg -> position.y;
  	phi_robot = (double) msg -> orientation.z;
  
  }

	//Action destructor
	
  ~MovementAction(void)
  {
  }

  void executeCB(const actionlib_movement::MovementGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    int i = 0;
		int radius =0.06; //meters

    // publish info to the console for the user
    ROS_INFO("%s: Executing, following path to %f, %f ", action_name_.c_str(), goal->final_point.position.x, goal->final_point.position.y);

    // start executing the action
    double **path_points = mypath.path_pts(goal->final_point.position.x, goal->final_point.position.y, x_robot, y_robot);
    while (i<=mypath.getNumber())
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        vel.linear.x = 0;
			 vel.angular.z = 0;
			pub_vel.publish(vel);
        success = false;
        break;
      }
      
      
				while (sqrt(pow(path_points[i][0]-x_robot,2)+pow(path_points[i][1]-y_robot,2))<= radius) i++;
	
			//Calculate angle and publish velocity
			double phi = phi_robot-atan2 ((path_points[i][1]-y_robot),(path_points[i][0]-x_robot));
			//publish angular_velocity = k * phi;
			 vel.linear.x = a;
			 vel.angular.z = k*phi;
			pub_vel.publish(vel);
			
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.current_point = feedback_.current_point;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }



};
#endif
