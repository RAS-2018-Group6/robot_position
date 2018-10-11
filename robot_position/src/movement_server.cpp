#include <ros/ros.h>
#include "path_creator.h"
#include "math.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

class movementAction{

	protected:

	ros::NodeHandle NH;
	actionlib::SimpleActionServer<????::movementAction> ActServ;
	std::string actionname;

	???::movementFeedback feedback;
	???::movementResult result;

public:

  movementAction(std::string name) 
    ActServ(NH, name, boost::bind(&movementAction::executeCB, this, _1), false),


    actionname(name)
  {
    ActServ.start();
  }
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	x_robot = msg.linear->x;
	y_robot = msg.linear->y;
	phi_robot = msg.angular->z;

	ros::Subscriber pose = NH.subscribe<geometry_msgs::Pose>("/odom", 1000, &movementActionNode::poseCallback, this);
	~movementAction(void)
		{
		}

	int follow(double **path, i){
			//See if (path [i] is outside of the circle)
			int radius =0.06; //meters
			while (sqrt(pow(path[i][0]-x_robot,2)+pow(path[i][1]-y_robot,2))<= radius) i++;
	
			//Calculate angle and publish velocity
			double phi = phi_robot-atan2 ((path[i][1]-y_robot)/(path[i][0]-x_robot));
			//publish angular_velocity = k * phi;
			 
			if (!points) {break;}
			return i;
		}

	 void executeCB(const ???::movementGoalConstPtr &goal)
		{
		  // helper variables
		  ros::Rate r(1);
		  bool success = true;

			path path_points;
		int i = 0;
		while (true){
			i = follow (path_points, i);

		 
		  {
		    // check that preempt has not been requested by the client
		    if (ActServ.isPreemptRequested() || !ros::ok())
		    {
		      ROS_INFO("%s: Preempted", action_name_.c_str());
		      // set the action state to preempted
		      ActServ.setPreempted();
		      success = false;
		      break;
		    }
		    
		    // publish the feedback
		    ActServ.publishFeedback(feedback);
		    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
		    r.sleep();
		  }

		  if(success)
		  {
		    result.sequence = feedback.sequence;
		    ROS_INFO("%s: Succeeded", action_name_.c_str());
		    // set the action state to succeeded
		    ActServ.setSucceeded(result);
		  }
		}


	};

	int main(int argc, char** argv)
{
  ros::init(argc, argv, "movement");
	
  movementAction movement("movement");
  ros::spin();

  return 0;
}


