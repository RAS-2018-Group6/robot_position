#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_movement/MovementAction.h>
#include "math.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
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
    bool do_once;
    bool got_map;
    actionlib_movement::MovementGoal goal;
    //MovementAction *movement_server;
    actionlib::SimpleActionClient<actionlib_movement::MovementAction> *movement_client;
    actionlib::SimpleActionClient<actionlib_movement::MovementAction> *backwards_movement_client;
    ros::ServiceClient gripper_client;
    arduino_servo_control::SetServoAngles srv;
    sensor_msgs::PointCloud exploration_targets;
    nav_msgs::OccupancyGrid map;
    bool current_point_done;
    int current_point_index;
    int N_FAILS;
    int MAX_FAILS;
    int sqrt_N_POINTS;
    int N_POINTS;


public:
    ros::NodeHandle n;
    ros::Subscriber sub_obstacle;
    ros::Subscriber sub_object;
    ros::Subscriber sub_map;
    ros::Publisher pub_targets;

    Brain(ros::NodeHandle node)
    {
        n = node;
        stopped = false;
        abort1 = false;
        do_once = true;
        got_map = false;
        current_point_done = true;

        N_FAILS = 0;
        MAX_FAILS = 2;
        sqrt_N_POINTS = 3;
        N_POINTS = pow(sqrt_N_POINTS,2);

        current_point_index = 0;


        //movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);
        //movement_server = new MovementAction("movement");
        sub_obstacle = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &Brain::obstacleCallback,this);
        sub_object = n.subscribe<geometry_msgs::PointStamped>("/found_object", 1, &Brain::objectCallback,this);
        sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/grid_map",1,&Brain::mapCallback,this);
        pub_targets = n.advertise<sensor_msgs::PointCloud>("/explore_targets",1);
        movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);
        backwards_movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("backwards_movement", true);

        gripper_client = n.serviceClient<arduino_servo_control::SetServoAngles>("arduino_servo_control/set_servo_angles");

        //initExploration();
    }
    ~Brain()
    {

    }
    void backOff()
    {
        N_FAILS++;
        movement_client->cancelGoal();
        ROS_INFO("Backing");
        //goal.final_point.position.x = 1;
        //goal.final_point.position.y = 1;
        //goal.final_point.orientation.z = 1;
        // add orientation

        ROS_INFO("BRAIN: Waiting for server.");
        backwards_movement_client->waitForServer();
        // TODO wait Duration(x), otherwise restart server.

        ROS_INFO("BRAIN: SERVER IS UP");
        backwards_movement_client->sendGoal(goal, boost::bind(&Brain::back_doneCb, this, _1, _2));
    }


    void explorationLoop()
    {
        // TODO: implement max number of fails before moving on.

        if (!current_point_done)
        {
            return;
        }else if (N_FAILS > MAX_FAILS)
        {
            current_point_index++;
        }
        else if (current_point_index == N_POINTS)
        {
            // return to starting position
            ROS_INFO("Done exploring.");
            current_point_done = false;
        }else
        {
            current_point_done = false;
            ROS_INFO("Moving to point %i",current_point_index+1);
            moveToPosition(exploration_targets.points[current_point_index].x, exploration_targets.points[current_point_index].y, 0);
        }


    }

    void initExploration()
    {
        ROS_INFO("Init exploration");
        if (!got_map)
        {
            ROS_INFO("No map..");
            return;
        }
        exploration_targets.points.resize(pow(sqrt_N_POINTS,2));
        exploration_targets.header.frame_id = "/map";


        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(-0.15, 0.15);

        float height = map.info.height;
        float width = map.info.width;
        int k = 0;
        for (int j = 1; j <= sqrt_N_POINTS; j++)
        {
            for (int i = 1; i <= sqrt_N_POINTS; i++)
            {
                exploration_targets.points[k].x = i*map.info.resolution*height/(sqrt_N_POINTS+1);
                exploration_targets.points[k].y = j*map.info.resolution*width/(sqrt_N_POINTS+1);
                //ROS_INFO("Point: (%f,%f)",exploration_targets.points[k].x,exploration_targets.points[k].y);
                k++;
            }
        }
        ROS_INFO("Evaluating points");
        // make sure points are valid
        int index;
        for (int i = 0; i < exploration_targets.points.size(); i++)
        {
            bool point_ok = 0;
            while(!point_ok)
            {
                index = mToCell(exploration_targets.points[i].y)*width+mToCell(exploration_targets.points[i].x);
                if(map.data[index] != 0)
                {
                    exploration_targets.points[i].x = exploration_targets.points[i].x + (float) dis(gen);
                    exploration_targets.points[i].y = exploration_targets.points[i].y + (float) dis(gen);
                }else
                {
                    point_ok = 1;
                }
            }
            //ROS_INFO("Point %i out of %i done.",i, exploration_targets.points.size());
        }
        ROS_INFO("Points done");
    }


    int mToCell(float x)
    {
        // converts x from meters to grid cell coordinate
        return (int) round(x/map.info.resolution);
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
            //do_once = false;
            //moveToPosition(2.2,0.3,0.0);

            ROS_INFO("BRAIN: Point %i explored. Moving on!", current_point_index);
            exploration_targets.points[current_point_index].x = 0;
            exploration_targets.points[current_point_index].y = 0;
            current_point_index++;
            N_FAILS = 0;
            //explorationLoop();
            current_point_done = true;

        }else
        {
            if (N_FAILS < MAX_FAILS)
            {
                N_FAILS++;
                current_point_done = true;
                //explorationLoop();
            }else
            {
                N_FAILS = 0;
                current_point_index++;
                current_point_done = true;
                //explorationLoop();
            }

        }
    }

    void back_doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_movement::MovementResultConstPtr& result) {
        ROS_INFO("BRAIN: backwards server responded with state [%s]", state.toString().c_str());

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            current_point_done = true;
            moveToPosition(goal.final_point.position.x, goal.final_point.position.y, goal.final_point.orientation.z);
            /*
            ROS_INFO("BRAIN: Point %i explored. Moving on!", current_point_index);
            exploration_targets.points[current_point_index].x = 0;
            exploration_targets.points[current_point_index].y = 0;
            current_point_index++;
            explorationLoop();
            */
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
           sleep(5);
           ROS_INFO  ("BRAIN: The goal has been cancelled. Need to move backwards.");
           backOff();


       }else if (msg -> data == false && stopped == true)
       {
           ROS_INFO("BRAIN: Obstacle not visible anymore.");
           stopped = false;

           //explorationLoop();

       }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        got_map = true;
        map = *map_msg;
        //ROS_INFO("got map res %f",map.info.resolution);
    }




};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "exploration_client");
  ros::NodeHandle n;

  Brain brain(n);
  ROS_INFO("Starting exploration mode.");
  //brain.moveToPosition(0.4,2.0,0.0); // Example action


  sleep(1);
  ros::spinOnce();
  sleep(1);

  ros::spinOnce();


  brain.initExploration();


  while(ros::ok())
  {
    brain.publishPoints();
    brain.explorationLoop();
    ros::spinOnce();
    //ROS_INFO("Spinning");
    sleep(0.1);
  }

  return 0;
}
