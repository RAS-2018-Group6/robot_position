

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> //Action library used from implementing simple actions
#include <actionlib_movement/MovementAction.h>
#include "math.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "create_path.cpp"


class MovementAction
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_movement::MovementAction> as_; // Action Server
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_movement::MovementFeedback feedback_;
  actionlib_movement::MovementResult result_;
  ros::Publisher pub_vel;
  ros::Subscriber sub_pose;

	//Path mypath;
    geometry_msgs::Twist vel;
    float a;
    float k;



public:


	//Action constructor

  MovementAction(std::string name) :
    as_(nh_, name, boost::bind(&MovementAction::executeCB, this, _1), false),
    action_name_(name)
  {

    pub_vel = nh_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    sub_pose = nh_.subscribe<geometry_msgs::Pose>("/pose", 1, &MovementAction::poseCallback, this);
    a = 0.1;
    k = 1;

    as_.start();
    //ros::Subscriber sub_pose = nh_.subscribe<geometry_msgs::Pose>("/pose", 10, &MovementAction::poseCallback, this);
    //ros::Publisher pub_vel = nh_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);

  }

  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){

        feedback_.current_point.position.x = msg -> position.x;
        feedback_.current_point.position.y = msg -> position.y;
        feedback_.current_point.orientation.z = msg -> orientation.z;
        as_.publishFeedback(feedback_);

  }

	//Action destructor

  ~MovementAction(void)
  {
  }


  void executeCB(const actionlib_movement::MovementGoalConstPtr &goal)
  {
    bool success = false;
    int i = 0;
    double radius =0.1; //meters


    // publish info to the console for the user
    ROS_INFO("%s: Executing, following path to %f, %f ", action_name_.c_str(), goal->final_point.position.x, goal->final_point.position.y);


    PathCreator my_path;
    std::vector<double> path_points;
    path_points = my_path.getPath(feedback_.current_point.position.x,feedback_.current_point.position.y,goal->final_point.position.x,goal->final_point.position.y);
    int nPoints = path_points.size() /2;


    ROS_INFO("Path points:");
    for (int i = 0; i<path_points.size(); i = i+2)
    {
        ROS_INFO("[%f, %f]",path_points[i], path_points[i+1]);
    }

    while ( i<= path_points.size() )
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
//        success = false;
//        break;
      }

      ROS_INFO("movement_class: Looping through all points.");
      for (i = 0; i<=path_points.size(); i = i+2)
      {
          double dist = sqrt(pow(path_points[i]-feedback_.current_point.position.x,2)+pow(path_points[i+1]-feedback_.current_point.position.y,2));

          if (dist >= radius)
          {
              ROS_INFO("Got point index %i for dist %f",i,dist);
              break;
          }
      }

      ROS_INFO("At position: [%f, %f]", feedback_.current_point.position.x, feedback_.current_point.position.y);
      ROS_INFO("Aiming at: [%f, %f]", path_points[i], path_points[i+1]);



        //Calculate angle and publish velocity
        double phi = feedback_.current_point.orientation.z-atan2 ((path_points[i]-feedback_.current_point.position.y),(path_points[i+1]-feedback_.current_point.position.x));
        vel.linear.x = a;
        vel.angular.z = k*phi;
        pub_vel.publish(vel);

      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes

      if(sqrt(pow(path_points[nPoints-2]-feedback_.current_point.position.x,2)+pow(path_points[nPoints-1]-feedback_.current_point.position.y,2))<= 0.05){
        success = true;
//        break;
      }
      //r.sleep();
  //    }

    if(success)
    {
      result_.current_point = feedback_.current_point;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    sleep(0.5);
    }
  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movement");
//  ROS_INFO("Movement_server: Creating MovementClass");
  MovementAction movement("movement");
//  ROS_INFO("Movement_server: spin()");
  ros::spin();

  return 0;
}
