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
#include <sensor_msgs/PointCloud.h>
#include <random>

class Brain
{
private:
    std::vector<ValuableObject> foundObjects;
    bool stopped;
    bool abort1;
    actionlib_movement::MovementGoal goal;
    //MovementAction *movement_server;
    actionlib::SimpleActionClient<actionlib_movement::MovementAction> *movement_client;
    ros::ServiceClient gripper_client;
    arduino_servo_control::SetServoAngles srv;
    sensor_msgs::PointCloud exploration_targets;
    int current_point_index;
    int N_FAILS;
    int MAX_FAILS;
    int N_POINTS;


public:
    ros::NodeHandle n;
    ros::Subscriber sub_obstacle;
    ros::Subscriber sub_object;
    ros::Publisher pub_targets;

    Brain(ros::NodeHandle node)
    {
        n = node;
        stopped = false;
        abort1 = false;

        N_FAILS = 0;
        MAX_FAILS = 3;
        N_POINTS = 10;
        current_point_index = 0;


        //movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);
        //movement_server = new MovementAction("movement");
        sub_obstacle = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &Brain::obstacleCallback,this);
        sub_object = n.subscribe<geometry_msgs::PointStamped>("/found_object", 1, &Brain::objectCallback,this);
        pub_targets = n.advertise<sensor_msgs::PointCloud>("/explore_targets",1);
        movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);

        gripper_client = n.serviceClient<arduino_servo_control::SetServoAngles>("arduino_servo_control/set_servo_angles");

        initExploration();
    }
    ~Brain()
    {

    }

    void explorationLoop()
    {
        // TODO: implement max number of fails before moving on.
        if (current_point_index == N_POINTS)
        {
            // return to starting position
        }else
        {
            moveToPosition(exploration_targets.points[current_point_index].x, exploration_targets.points[current_point_index].y, 0);
        }


    }

    void initExploration()
    {
        exploration_targets.points.resize(N_POINTS);
        exploration_targets.header.frame_id = "/map";

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(0.0, 2.4);

        for (int i = 0; i < N_POINTS; i++)
        {
            exploration_targets.points[i].x = (float) dis(gen);
            exploration_targets.points[i].y = (float) dis(gen);
        }
    }

    void publishPoints()
    {
        pub_targets.publish(exploration_targets);
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


    void doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_movement::MovementResultConstPtr& result) {
        ROS_INFO("BRAIN: server responded with state [%s]", state.toString().c_str());

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("BRAIN: Point %i explored. Moving on!", current_point_index);
            exploration_targets.points[current_point_index].x = 0;
            exploration_targets.points[current_point_index].y = 0;
            current_point_index++;
            explorationLoop();
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



    void objectCallback(const geometry_msgs::PointStamped::ConstPtr& msg){

       double x = msg->point.x;
       double y = msg->point.y;
       int type = (int) msg->point.z;

       for (int i = 0; i < foundObjects.size(); i++)
       {
           if (foundObjects[i].x == x && foundObjects[i].y == y)
           {
               // object already seen
               return;
           }
       }
       foundObjects.push_back(*(new ValuableObject(x,y,type)));
       if (abort1 == false)
       {
         movement_client->cancelGoal();
         ROS_INFO  ("BRAIN: The goal has been cancelled.");
         moveToPosition(x,y,0);
     }

    }

    void obstacleCallback(const std_msgs::Bool::ConstPtr& msg)
    {
       if (msg -> data == true && stopped == false)
       {
           stopped = true;
           movement_client->cancelGoal();
           ROS_INFO  ("BRAIN: The goal has been cancelled. Need to move backwards.");
           // TODO: Move backwards

       }else if (msg -> data == false && stopped == true)
       {
           ROS_INFO("BRAIN: Restarting goal.");
           stopped = false;
           //moveToPosition(goal.final_point.position.x, goal.final_point.position.y, goal.final_point.orientation.z);
           explorationLoop();

       }
    }




};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "exploration_client");
  ros::NodeHandle n;

  Brain brain(n);
  //brain.moveToPosition(2.3,0.6,0.0); // Example action
  brain.explorationLoop();
  ros::spin();

  return 0;
}
