#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"




class pose_tf{

protected:

  ros::NodeHandle nh_;
/*	double x_rob;
	double y_rob;
	double phi_rob;
	nav_msgs/Odometry msg;*/
	


public:

	geometry_msgs::Pose pose;
	double x_rob;
	double y_rob;
	double phi_rob;
	nav_msgs::Odometry msg;
	
	void subsCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
   x_rob = msg -> pose.pose.position.x;
   y_rob = msg -> pose.pose.position.y;
   phi_rob = msg -> pose.pose.orientation.z;
   
	}
	
	ros::Subscriber sub = nh_.subscribe("/odom", 1000, &pose_tf::subsCallback, this);
  ros::Publisher pub = nh_.advertise<geometry_msgs::Pose>("/pose", 1000);


  
};

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "transformer");
	
	pose_tf mypose;
	
  ros::Rate loop_rate(10);
  
	while(ros::ok()){

		
		mypose.pose.position.x = mypose.x_rob;
		mypose.pose.position.y = mypose.y_rob;
		mypose.pose.orientation.z = mypose.phi_rob;
		
		mypose.pub.publish(mypose.pose);
	

		ros::spinOnce();
		loop_rate.sleep();

	}
  return 0;


}


