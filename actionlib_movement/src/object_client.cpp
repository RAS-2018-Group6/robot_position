#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_movement/MovementAction.h>
#include "math.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include "object.cpp"
#include <vector>
#include "arduino_servo_control/SetServoAngles.h"

class Brain
{
public:
    ros::NodeHandle n;
    ros::Subscriber sub_obstacle;
    ros::Subscriber sub_object;

    Brain(ros::NodeHandle node)
    {
        n = node;
        stopped = false;
        abort1 = false;
	      do_once = true;
	catching_object = false;
	average_position = 0;
	object_position_x = 0.0;
	object_position_y = 0.0;
        //movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);
        //movement_server = new MovementAction("movement");
        sub_obstacle = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &Brain::obstacleCallback,this);
        sub_object = n.subscribe<geometry_msgs::PointStamped>("/found_object", 1, &Brain::objectCallback,this);
        movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);

        gripper_client = n.serviceClient<arduino_servo_control::SetServoAngles>("arduino_servo_control/set_servo_angles");
	       gripperDown();
	        sleep(1);
	         gripperUp();
    }
    ~Brain()
    {

    }

 void gripperDown()
 {
	   srv.request.angle_servo_1 = 0;

	   if(gripper_client.call(srv)){
		     ROS_INFO("Gripper down");
	   }
	   else{
		     ROS_INFO("Failed to lower gripper");
	   }
}

void gripperUp()
{
	srv.request.angle_servo_1 = 90;

	if(gripper_client.call(srv)){
		ROS_INFO("Gripper up");
	}
	else{
		ROS_INFO("Failed to rise gripper");
	}
}


    void objectCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(!catching_object){
		if(average_position == 0){
			object_position_x = msg->point.x;
       			object_position_y = msg->point.y;
			average_position++;
			movement_client->cancelGoal();
       			ROS_INFO  ("BRAIN: The goal has been cancelled.");
		}
		else{
			object_position_x = (object_position_x + msg->point.x)/2;
       			object_position_y = (object_position_y + msg->point.y)/2;
			average_position++;
		}
	
	
  

       
       ROS_INFO ("Detected object position: (%f,%f)",object_position_x,object_position_y);
	if(average_position > 40){
		catching_object = true;
       		moveToPosition(object_position_x,object_position_y,0);
	}
	
	
	}
       

    }

    void obstacleCallback(const std_msgs::Bool::ConstPtr& msg)
    {
       if (msg -> data == true && stopped == false)
       {
           stopped = true;
           movement_client->cancelGoal();
           ROS_INFO  ("BRAIN: The goal has been cancelled.");

       }else if (msg -> data == false && stopped == true)
       {
           ROS_INFO("BRAIN: Restarting goal.");
           stopped = false;
           moveToPosition(goal.final_point.position.x, goal.final_point.position.y, goal.final_point.orientation.z);
       }
    }


    void doneCb(const actionlib::SimpleClientGoalState& state,
        const actionlib_movement::MovementResultConstPtr& result) {
        ROS_INFO("BRAIN: server responded with state [%s]", state.toString().c_str());

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED && do_once)
        {
            gripperDown();
		        do_once = false;
            moveToPosition(2.2,0.2,0.0);
            //sleep(2);
            //moveToObject(0,0,0);
        }
    }




    void moveToObject()
    {
        if (foundObjects.size() == 0)
        {
            ROS_INFO("BRAIN: No objects are found. Could not move to object position.");
            return;
        }

        //movement_server = new MovementAction("movement");
        //actionlib_movement::MovementGoal goal;

        goal.final_point.position.x = foundObjects[0].x;// x_dest;
        goal.final_point.position.y = foundObjects[0].y; //y_dest;

        ROS_INFO("BRAIN: Waiting for server.");
        movement_client->waitForServer();
        // TODO wait Duration(x), otherwise restart server.

        ROS_INFO("BRAIN: SERVER IS UP");
        movement_client->sendGoal(goal, boost::bind(&Brain::doneCb, this, _1, _2));
    }

    void moveToPosition(float x, float y, float z)
    {
        //movement_server = new MovementAction("movement");
        //movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);


        goal.final_point.position.x = x;
        goal.final_point.position.y = y;
        goal.final_point.orientation.z = z;
        // add orientation

        ROS_INFO("BRAIN: Waiting for server.");
        movement_client->waitForServer();
        // TODO wait Duration(x), otherwise restart server.

        ROS_INFO("BRAIN: SERVER IS UP");
        movement_client->sendGoal(goal, boost::bind(&Brain::doneCb, this, _1, _2));
    }


private:
    std::vector<ValuableObject> foundObjects;
    bool stopped;
    bool abort1;
    bool catching_object;
    actionlib_movement::MovementGoal goal;
    //MovementAction *movement_server;
    actionlib::SimpleActionClient<actionlib_movement::MovementAction> *movement_client;
    ros::ServiceClient gripper_client;
    arduino_servo_control::SetServoAngles srv;
    bool do_once;
    int average_position;
    double object_position_x , object_position_y;


};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "send_movement");
  ros::NodeHandle n;

  Brain brain(n);
  brain.moveToPosition(0.6,1.6,0.0); // Example action
  ros::spin();

  return 0;
}
