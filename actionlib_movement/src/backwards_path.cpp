

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <deque>

class PathMemory
{
private:
    ros::NodeHandle n;
    std::deque<float> past_x;
    std::deque<float> past_y;
    nav_msgs::Path path_msg;
    tf::TransformListener *tf_listener;
    tf::StampedTransform base_tf;
    float path_distance;
    float current_path_distance;
    ros::Publisher pub_path;


public:


    PathMemory(ros::NodeHandle node)
    {
        n = node;
        tf_listener = new tf::TransformListener();

        path_distance = 0.5;
        path_msg.header.frame_id = "/map";

        pub_path = n.advertise<nav_msgs::Path>("/past_path", 1);
        tf_listener->waitForTransform("/map","/base_link",ros::Time(0), ros::Duration(5));

    }

    ~PathMemory(void)
    {
    }

    void getPath()
    {

        float x,y, x_prev, y_prev, path_length;
        ros::Time start_time = ros::Time::now();

        tf_listener->lookupTransform("/map", "/base_link",ros::Time(0), base_tf);
        start_time = base_tf.stamp_;

        path_length = 0;
        int i = 0;
        while(path_length < 0.2)
        {
            // get transform 0.1 seconds back in time
            tf_listener->lookupTransform("/map", "/base_link",start_time-ros::Duration(i*0.1), base_tf);

            x = base_tf.getOrigin().x();
            y = base_tf.getOrigin().y();

            if (past_x.empty())
            {
                ROS_INFO("First element added");
                past_x.push_back(x);
                past_y.push_back(y);

            }else if (sqrt( pow(past_x.back()-x,2) + pow(past_y.back()-y,2) ) > 0.01)
            {
                // Only add if this point is further than 1 cm away from the last point.
                path_length += sqrt( pow(past_x.back()-x,2) + pow(past_y.back()-y,2) );
                past_x.push_back(x);
                past_y.push_back(y);

                ROS_INFO("Added element");

            }
            i++;
        }

        path_msg.poses.resize(past_x.size());
        for (int i = 0; i < past_x.size(); i++)
        {
            //current_path_distance += sqrt( pow(past_x[i]-x_prev,2) + pow(past_y[i]-y_prev,2) );
            path_msg.poses[i].pose.position.x = past_x[i];
            path_msg.poses[i].pose.position.y = past_y[i];
        }
        pub_path.publish(path_msg);

    }

    void memoryLoop()
    {
        tf_listener->lookupTransform("/map", "/base_link",ros::Time(0), base_tf);

        float x = base_tf.getOrigin().x();
        float y = base_tf.getOrigin().y();

        if (past_x.empty())
        {
            ROS_INFO("First element added");
            past_x.push_back(x);
            past_y.push_back(y);

        }else if (sqrt( pow(past_x.back()-x,2) + pow(past_y.back()-y,2) ) < 0.01)
        {
            ROS_INFO("Standing still");
            return;
        }else if (current_path_distance < path_distance)
        {
            ROS_INFO("Added to path");
            past_x.push_back(x);
            past_y.push_back(y);
        }else
        {
            ROS_INFO("Push pop path");
            past_x.pop_front();
            past_y.pop_front();

            past_x.push_back(x);
            past_y.push_back(y);
        }

        float y_prev = past_x[0];
        float x_prev = past_y[0];
        current_path_distance = 0;
        path_msg.poses.resize(past_x.size());
        for (int i = 0; i < past_x.size(); i++)
        {
            current_path_distance += sqrt( pow(past_x[i]-x_prev,2) + pow(past_y[i]-y_prev,2) );
            path_msg.poses[i].pose.position.x = past_x[i];
            path_msg.poses[i].pose.position.y = past_y[i];
            x_prev = past_x[i];
            y_prev = past_y[i];
        }

    }

    void publishPath()
    {
        pub_path.publish(path_msg);
    }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_memory");
  ros::NodeHandle node;
  ros::Rate loop_rate(5);

  PathMemory pm(node);
  ROS_INFO("Sleeping");
  sleep(5);
  pm.getPath();
  ROS_INFO("Path done");


  while(ros::ok())
  {
      //ros::spinOnce();
      //pm.memoryLoop();
      pm.publishPath();
      loop_rate.sleep();
  }


  return 0;
}
