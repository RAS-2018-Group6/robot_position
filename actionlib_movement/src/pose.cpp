#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

double x_rob;
double y_rob;
double phi_rob;


void subsCallback(const geometry_msgs::Odometry::ConstPtr& msg)
{
  global x_rob = msg -> pose.pose.position.x;
  global y_rob = msg -> pose.pose.position.y;
  global phi_rob = msg -> pose.pose.orientation.z;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/odom", 1000, subsCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Pose>("/pose", 1000);
  ros::Rate loop_rate(10);
  
  msg.position.x = x_rob;
  msg.position.y = y_rob;
  msg.orientation.z = w_rob;
  
  pub.publish(msg);

  ros::spin();

  return 0;


}


