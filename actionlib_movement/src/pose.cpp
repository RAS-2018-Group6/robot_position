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

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
   x_rob = msg -> pose.pose.position.x;
   y_rob = msg -> pose.pose.position.y;
   phi_rob = msg -> pose.pose.orientation.z;

	}

        ros::Subscriber sub_odom = nh_.subscribe("/odom", 1, &pose_tf::odomCallback, this);
  ros::Publisher pub_pose = nh_.advertise<geometry_msgs::Pose>("/pose", 1);



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

		mypose.pub_pose.publish(mypose.pose);


		ros::spinOnce();
		loop_rate.sleep();

	}
  return 0;


}
