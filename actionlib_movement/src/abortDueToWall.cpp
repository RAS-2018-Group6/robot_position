
//Takes in robot pose and lidar measurements
//Aborts path if too many (m) readings of obstacle ahead

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
//#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"

//float heading; not needed for now
//float xr;
//float yr;
int k = 0; //counter
int m = 5; //number of measurements within safety distance before abortion
std_msgs::Bool wallDetected; //start by assuming no wall



/* void readPose(const geometry_msgs::Pose  pose)
{
	heading = (float) pose.orientation.z; not needed for now
	xr = (float) pose.position.x;   //Does Quaternion instead of Point make a differens???
	yr = (float) pose.position.y;

}
*/

void readScan(const sensor_msgs::LaserScan scan)
{

//	float scanData[360];// = scan.ranges;
//I guess the list is organized as first element corresponds to 1 degree and so on


		for(int i=0; i < 30; i++){
			if(scan.ranges[i] <= 0.45){k++;} //safety distance of 0.3 meters
		}
		k--; //Forget one measurement to not build up over time
		for(int i=330; i < 360; i++){
			if(scan.ranges[i] <= 0.45){k++;} //safety distance of 0.3 meters
		}
		k--; //Forget one measurement to not build up over time

    if(k<m){wallDetected.data = false;}
    if(k>m){
      wallDetected.data = true;
      k = 0; //restart counter
    }

//	ROS_INFO("\nError_1: %f, Int_error_1: %f\nError_2: %f, Int_error_2: %f\n", error1, int_error1, error2, int_error2);


}


int main(int argc, char **argv)
{
	ros::init(argc,argv, "abortDueToWall");
	ros::NodeHandle n;

//	ros::Subscriber pose = n.subscribe("/Pose", 1000, readPose);
	ros::Subscriber scan = n.subscribe("/scan", 1000, readScan);

	ros::Publisher pub_bool = n.advertise<std_msgs::Bool>("/wall_detected",1);

	wallDetected.data = false;

	ros::Rate loop_rate(20);

	while (ros::ok())
	{

		std_msgs::Bool b;


		b.data = wallDetected.data;

		//RAS_INFO("The demanded speeds are %d and %d", p.PWM1, p.PWM2);

		pub_bool.publish(b);

		ros::spinOnce();
		loop_rate.sleep();

	}

}
