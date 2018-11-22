

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> //Action library used from implementing simple actions
#include <actionlib_movement/MovementAction.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
//#include "create_path.cpp"
#include "path_planner.cpp"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


class MovementAction
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actionlib_movement::MovementAction> as_; // Action Server
    std::string action_name_;

    actionlib_movement::MovementFeedback feedback_;
    actionlib_movement::MovementResult result_;
    //actionlib_movement::MovementGoal goal_;
    ros::Publisher pub_vel;
    ros::Publisher pub_path;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_map_;


    tf::Pose pose;
    geometry_msgs::Twist vel;
    float a;
    float k;
    float heading;

    float theta;

    int nRows_, nColumns_;
		float map_resolution_;
		std::vector<signed char> data_;
    std::vector<float> nextgoal;




public:


    MovementAction(std::string name) :
        as_(nh_, name, boost::bind(&MovementAction::executeCB, this, _1), false),
        action_name_(name)
        {

        pub_vel = nh_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        pub_path = nh_.advertise<nav_msgs::Path>("/path_publish", 10000);
        sub_pose = nh_.subscribe<nav_msgs::Odometry>("/particle_position", 1, &MovementAction::poseCallback, this);
        sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/smooth_map",1,&MovementAction::mapCallback,this);


        a = 0.1; // linear velocity
        k = 0.5; // factor for angular velocity
        theta = 0.0;


        as_.start();
    }

    ~MovementAction(void)
    {
    }


    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf::poseMsgToTF(msg->pose.pose, pose);
        feedback_.current_point.position.x = msg -> pose.pose.position.x;
        feedback_.current_point.position.y = msg -> pose.pose.position.y;
        feedback_.current_point.orientation.z = msg-> pose.pose.orientation.z;
        heading = tf::getYaw(pose.getRotation());
	      if (heading < 0){
		      heading = heading + 2 * M_PI;
	      }


        as_.publishFeedback(feedback_);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
		{
				//ROS_INFO("Map Callback");
				nRows_ = msg->info.height;
				nColumns_ = msg->info.width;
				map_resolution_ = msg->info.resolution;
				data_ = msg-> data;

		}

    void PathRos(std::vector<float> path){ //Publish the path points in rviz
	     int j = 0;
       int i= 0;
	      nav_msgs::Path path_Ros;
        pub_path.publish(path_Ros); //Without this line, the message is not published
        path_Ros.poses.resize(path.size()/2);
        path_Ros.header.frame_id = "/map";
	       if (path.size()>2){
           for (i = 0; i < path.size()-3; i+=2){
  	            path_Ros.poses[j].pose.position.x = path[i];
  	            path_Ros.poses[j].pose.position.y = path[i+1];
                path_Ros.poses[j].pose.position.z = 0;
  	            path_Ros.poses[j].pose.orientation.x = 0;
  	            path_Ros.poses[j].pose.orientation.y = 0;
  	            path_Ros.poses[j].pose.orientation.z = 0;
  	            path_Ros.poses[j].pose.orientation.w = atan2((path[i+3]-path[i+1]),(path[i+2]-path[i]));
                //ROS_INFO("x = %f, y = %f, w = %f",path_Ros.poses[j].pose.position.x, path_Ros.poses[j].pose.position.y, path_Ros.poses[j].pose.orientation.w);
  	            j ++;
		        }
            path_Ros.poses[j].pose.position.x = path[i];
            path_Ros.poses[j].pose.position.y = path[i+1];
            path_Ros.poses[j].pose.position.z = 0;
	          path_Ros.poses[j].pose.orientation.x = 0;
            path_Ros.poses[j].pose.orientation.y = 0;
            path_Ros.poses[j].pose.orientation.z = 0;
            path_Ros.poses[j].pose.orientation.w = path_Ros.poses[j-1].pose.orientation.w; //The orientation of the last point is the same as the previous one
            //ROS_INFO("x = %f, y = %f, w = %f",path_Ros.poses[j].pose.position.x, path_Ros.poses[j].pose.position.y, path_Ros.poses[j].pose.orientation.w);
          }
          pub_path.publish(path_Ros);
          ROS_INFO("The path has been published");
    }


    void executeCB(const actionlib_movement::MovementGoalConstPtr &goal)
    {
        float distance_to_goal, phi, dist;
        bool success = true;
        int i = 0;
        float radius = 0.02; //meters
        PathCreator current_path (nRows_);
        std::vector<float> path_points;
        ROS_INFO("SERVER: Got goal position: [%f, %f]",goal->final_point.position.x,goal->final_point.position.y);
      //  path_points = current_path.getPath(0.2,0.2,goal->final_point.position.x,goal->final_point.position.y, nRows_, nColumns_,map_resolution_,data_);
        path_points = current_path.getPath(feedback_.current_point.position.x,feedback_.current_point.position.y,goal->final_point.position.x,goal->final_point.position.y, nRows_, nColumns_,map_resolution_,data_);
        PathRos(path_points); //Publish the path in rviz

      //  path_points = current_path.getPath(feedback_.current_point.position.x,feedback_.current_point.position.y,goal->final_point.position.x,goal->final_point.position.y, nRows_, nColumns_,map_resolution_,data_);
        int nPoints = path_points.size() / 2;




        /*
        ROS_INFO("Path points:");
        for (int ind = 0; ind<path_points.size(); ind = ind+2)
        {
            ROS_INFO("[%f, %f]",path_points[ind], path_points[ind+1]);
        }
        */


        ROS_INFO("number of path points: %i",path_points.size());

        while ( i<= path_points.size() )
        {
          //ROS_INFO("Entered while Loop");
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("SERVER: %s: Server preempted ", action_name_.c_str());
		ROS_INFO("CURRENT POSITION: %f, %f", feedback_.current_point.position.x, feedback_.current_point.position.y);
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_vel.publish(vel);
                as_.publishFeedback(feedback_);
                as_.setPreempted();
                success = false;
                break;

            }

            //ROS_INFO("movement_class: Looping through all points.");
            for (i; i<=path_points.size(); i = i+2)
            {
                dist = sqrt(pow(path_points[i]-feedback_.current_point.position.x,2)+pow(path_points[i+1]-feedback_.current_point.position.y,2));
                if (dist >= radius)
                {
                    //ROS_INFO("Got point index: [%i / %i], for distance: %f",i,nPoints,dist);
                    break;
                }else if(i == path_points.size()-2)
                {
                    break;
                }
            }


          //  ROS_INFO("At position: [%f, %f]", feedback_.current_point.position.x, feedback_.current_point.position.y);
          //  ROS_INFO("Aiming at: [%f, %f]", path_points[i], path_points[i+1]);

	        //theta = atan2 ((path_points[i+1]-feedback_.current_point.position.y),(path_points[i]-feedback_.current_point.position.x));

	          theta = atan2 ((path_points[i+1]-feedback_.current_point.position.y),(path_points[i]-feedback_.current_point.position.x));

            phi = -heading + theta;
            if (phi > M_PI)
            {
              phi = phi - 2*M_PI;
            }
            else if(phi < -M_PI){
              phi = phi + 2*M_PI;
            }
	    //ROS_INFO("HEADING: %f, THETA: %f, PHI: %f",heading,theta,phi);


            vel.linear.x = a*pow( (M_PI-fabs(phi)) / M_PI ,2);
            vel.angular.z = k*phi;
            pub_vel.publish(vel);

            as_.publishFeedback(feedback_);

            distance_to_goal = sqrt(pow((goal->final_point.position.x)-feedback_.current_point.position.x,2)+pow((goal->final_point.position.y)-feedback_.current_point.position.y,2));

          //ROS_INFO("Current distance to goal: %f",distance_to_goal);

            if(distance_to_goal<= 0.03)
            {
                // set desired orientation;
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_vel.publish(vel);
                break;
            }
        }
	
	

        if(success)
        {
            result_.current_point = feedback_.current_point;
            ROS_INFO("SERVER: %s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
            return;
        }

    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "movement");
//  ROS_INFO("Movement_server: Creating MovementClass");
  MovementAction movement("movement");
  ROS_INFO("Movement_server: spin()");
  ros::spin();

  return 0;
}
