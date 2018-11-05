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
        //movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);
        //movement_server = new MovementAction("movement");
        sub_obstacle = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &Brain::obstacleCallback,this);
        sub_object = n.subscribe<geometry_msgs::PointStamped>("/found_object", 1, &Brain::objectCallback,this);
        movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);

    }
    ~Brain()
    {

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

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("BRAIN: Sending new goal. Lets go again!");
            sleep(2);
            moveToPosition(0,0,0);
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
    actionlib_movement::MovementGoal goal;
    //MovementAction *movement_server;
    actionlib::SimpleActionClient<actionlib_movement::MovementAction> *movement_client;


};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "send_movement");
  ros::NodeHandle n;

  Brain brain(n);
  brain.moveToPosition(0.2,1.5,0.0); // Example action
  ros::spin();

  return 0;
}
