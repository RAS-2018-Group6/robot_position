

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
    tf::TransformListener *tf_listener;
    tf::StampedTransform base_tf;


    tf::Pose pose;
    geometry_msgs::Twist vel;
    float factor_linear;
    float factor_angular;
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
        sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/grid_map",1,&MovementAction::mapCallback,this);

        tf_listener = new tf::TransformListener();
        factor_linear = 0.1; // linear velocity
        factor_angular = 0.5; // factor for angular velocity
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

    std::vector<float>  getPath()
    {
        //ROS_INFO("Calculate path");
        float x,y, x_prev, y_prev, path_length;
        std::vector<float> return_points;
        ros::Time start_time = ros::Time::now();

        tf_listener->lookupTransform("/map", "/base_link",ros::Time(0), base_tf);
        start_time = base_tf.stamp_;

        path_length = 0;
        int nPoints = 0;
        int i = 0; // loop variable for tf time
        try{
          while(path_length < 0.2)
          {
              // get transform 0.1 seconds back in time
              tf_listener->lookupTransform("/map", "/base_link",start_time-ros::Duration(i*0.1), base_tf);

              x = base_tf.getOrigin().x();
              y = base_tf.getOrigin().y();

              if (return_points.empty())
              {
                  //ROS_INFO("First element added");
                  nPoints = nPoints + 2;
                  return_points.push_back(x);
                  return_points.push_back(y);

              }else if (sqrt( pow(return_points[nPoints-2]-x,2) + pow(return_points[nPoints-1]-y,2) ) > 0.04)
              {
                  // Only add if this point is further than 1 cm away from the last point.
                  path_length += sqrt( pow(return_points[nPoints-2]-x,2) + pow(return_points[nPoints-1]-y,2) );
                  return_points.push_back(x);
                  return_points.push_back(y);
                  nPoints = nPoints + 2;
                  //ROS_INFO("Added element");
              }
              i++;
          }
        }catch(tf::TransformException ex)
        {
          ROS_INFO("Exception caught in backwards server.");
          return_points.resize(2);
          return return_points;
        }

        //ROS_INFO("Lenght: %f", path_length);

        return return_points;
    }


    std::vector<float>  getStraightPath()
    {
        std::vector<float> return_points;

        return_points.push_back(feedback_.current_point.position.x);
        return_points.push_back(feedback_.current_point.position.y);
        return_points.push_back(feedback_.current_point.position.x - 0.15* cos(heading));
        return_points.push_back(feedback_.current_point.position.y - 0.15* sin(heading));

        return return_points;
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
          //ROS_INFO("The path has been published");
    }


    void executeCB(const actionlib_movement::MovementGoalConstPtr &goal)
    {
        float distance_to_goal, phi, dist;
        bool success = true;
        int i = 0;
        float radius = 0.08; //meters
        PathCreator current_path (nColumns_);
        std::vector<float> path_points;
        //ROS_INFO("SERVER: Backwards = %s", goal->backwards ? "true" : "false" );
        if (goal->backwards == 1)
        {
          ROS_INFO("SERVER: Got backwards goal");
          path_points = getStraightPath();
          factor_linear = -0.08;
          factor_angular = -0.5;
        }else
        {
          ROS_INFO("SERVER: Got goal position: [%f, %f]",goal->final_point.position.x,goal->final_point.position.y);
          path_points = current_path.getPath(feedback_.current_point.position.x,feedback_.current_point.position.y,goal->final_point.position.x,goal->final_point.position.y, nRows_, nColumns_,map_resolution_,data_);
          factor_linear = 0.1; // linear velocity
          factor_angular = 0.5; // factor for angular velocity
        }

        PathRos(path_points); //Publish the path in rviz
        int nPoints = path_points.size() / 2;
        float required_dist = goal->min_distance;
        ROS_INFO("SERVER: Required distance to goal: %f",required_dist);

        //ROS_INFO("number of path points: %i",path_points.size());
        while ( i<= path_points.size() )
        {
            if (nPoints <= 1){
              ROS_INFO("SERVER: no path, %s: Server aborted ", action_name_.c_str());
              //ROS_INFO("CURRENT POSITION: %f, %f", feedback_.current_point.position.x, feedback_.current_point.position.y);
              //ROS_INFO("No path found to goal position!");
              vel.linear.x = 0;
              vel.angular.z = 0;
              pub_vel.publish(vel);
              as_.publishFeedback(feedback_);
              as_.setAborted();
              success = false;
              break;
            }
          //ROS_INFO("Entered while Loop");
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("SERVER: %s: Server preempted by client ", action_name_.c_str());
		            //ROS_INFO("CURRENT POSITION: %f, %f", feedback_.current_point.position.x, feedback_.current_point.position.y);
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

	          theta = atan2 ((path_points[i+1]-feedback_.current_point.position.y),(path_points[i]-feedback_.current_point.position.x));
            phi = -heading + theta;

            if (goal->backwards == 1)
            {
              phi = -phi + M_PI;

            }

            if (phi > M_PI)
            {
              phi = phi - 2*M_PI;
            }
            else if(phi < -M_PI){
              phi = phi + 2*M_PI;
            }

            vel.linear.x = factor_linear*pow( (M_PI-fabs(phi)) / M_PI ,2);
            vel.angular.z = factor_angular*phi;
            pub_vel.publish(vel);

            as_.publishFeedback(feedback_);

            distance_to_goal = sqrt(pow(path_points[nPoints*2-2]-feedback_.current_point.position.x,2)+pow(path_points[nPoints*2-1]-feedback_.current_point.position.y,2));

          //ROS_INFO("Current distance to goal: %f",distance_to_goal);
            //float required_dist = goal->min_distance;
            //ROS_INFO("SERVER: Required distance to goal: %f",required_dist);
            if(distance_to_goal<= required_dist)
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
