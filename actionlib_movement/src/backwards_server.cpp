

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> //Action library used from implementing simple actions
#include <actionlib_movement/MovementAction.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
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
    ros::Publisher pub_vel;
    ros::Publisher pub_path;
    ros::Subscriber sub_pose;
    tf::TransformListener *tf_listener;
    tf::StampedTransform base_tf;

    tf::Pose pose;
    geometry_msgs::Twist vel;
    nav_msgs::Path path_msg;
    float factor_linear;
    float factor_angular;
    float heading;
    float theta;
    std::vector<float> nextgoal;




public:


    MovementAction(std::string name) :
        as_(nh_, name, boost::bind(&MovementAction::executeCB, this, _1), false),
        action_name_(name)
    {
        ROS_INFO("Constructor start");
        pub_vel = nh_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        pub_path = nh_.advertise<nav_msgs::Path>("/path_publish", 1);
        sub_pose = nh_.subscribe<nav_msgs::Odometry>("/particle_position", 1, &MovementAction::poseCallback, this);
        tf_listener = new tf::TransformListener();

        factor_linear = -0.08;
        factor_angular = -0.5;
        theta = 0.0;
        ROS_INFO("Starting server.");
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
        /*else if (heading > 2*M_PI)
        {
          heading = heading - 2*M_PI;
        }*/

        as_.publishFeedback(feedback_);
    }

    std::vector<float>  getPath()
    {
        ROS_INFO("Calculate path");
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
                  ROS_INFO("First element added");
                  nPoints = nPoints + 2;
                  return_points.push_back(x);
                  return_points.push_back(y);

              }else if (sqrt( pow(return_points[nPoints-2]-x,2) + pow(return_points[nPoints-1]-y,2) ) > 0.05)
              {
                  // Only add if this point is further than 1 cm away from the last point.
                  path_length += sqrt( pow(return_points[nPoints-2]-x,2) + pow(return_points[nPoints-1]-y,2) );
                  return_points.push_back(x);
                  return_points.push_back(y);
                  nPoints = nPoints + 2;
                  ROS_INFO("Added element");
              }
              i++;
          }
        }catch(tf::TransformException ex)
        {
          ROS_INFO("Exception caught in backwards server.");
          return_points.resize(2);
          return return_points;
        }

        ROS_INFO("Lenght: %f", path_length);

        path_msg.poses.resize(nPoints/2);
        path_msg.header.frame_id = "/map";
        //return_points.resize(2*past_x.size());

        int j = 0;
        for (int i = 0; i < nPoints; i=i+2)
        {
            path_msg.poses[j].pose.position.x = return_points[i];
            path_msg.poses[j].pose.position.y = return_points[i+1];
            j++;
        }
        pub_path.publish(path_msg);
        //delete tf_listener;

        return return_points;
    }


    void executeCB(const actionlib_movement::MovementGoalConstPtr &goal)
    {

        float distance_to_goal, phi, dist;
        bool success = true;
        int i = 0;
        float radius = 0.05; //meters

        std::vector<float> path_points = getPath();
        int nPoints = path_points.size() / 2;

        ROS_INFO("number of path points: %i",path_points.size());

        while ( i<= path_points.size() )
        {
            if (path_points.size() == 2)
            {
              vel.linear.x = 0;
              vel.angular.z = 0;
              pub_vel.publish(vel);
              as_.publishFeedback(feedback_);
              as_.setAborted();
              success = false;
              break;
            }else if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("SERVER: %s: Server preempted ", action_name_.c_str());
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

            theta = atan2 ((path_points[i+1]-feedback_.current_point.position.y),(path_points[i]-feedback_.current_point.position.x));

            phi = -heading + theta;
            phi = -phi + M_PI;
            if (phi > M_PI)
            {
                phi = phi - 2*M_PI;
            }
            else if(phi < -M_PI){
                phi = phi + 2*M_PI;
            }
            //ROS_INFO("HEADING: %f, THETA: %f, PHI: %f",heading,theta,phi);

            vel.linear.x = factor_linear*pow( (M_PI-fabs(phi)) / M_PI ,2);
            vel.angular.z = factor_angular*phi;
            pub_vel.publish(vel);

            as_.publishFeedback(feedback_);

            distance_to_goal = sqrt(pow((path_points[nPoints*2-2])-feedback_.current_point.position.x,2)+pow((path_points[nPoints*2-1])-feedback_.current_point.position.y,2));
            //ROS_INFO("Current distance to goal: %f",distance_to_goal);

            if(distance_to_goal<= 0.02)
            {
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
    ros::init(argc, argv, "backwards_movement");
    MovementAction movement("backwards_movement");
    ROS_INFO("Backwards movement server up");
    ros::spin();

    return 0;
}
