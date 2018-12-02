#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_movement/MovementAction.h>
#include "math.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "object.cpp"
#include <vector>
#include "arduino_servo_control/SetServoAngles.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <random>
#include <string>
#include <cmath>

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
    ros::ServiceClient gripper_client;
    arduino_servo_control::SetServoAngles srv;
    sensor_msgs::PointCloud exploration_targets;
    nav_msgs::OccupancyGrid map;
    std_msgs::String sound_msg;
    bool current_action_done;
    int current_point_index;
    int N_FAILS;
    int MAX_FAILS;
    int sqrt_N_POINTS;
    int N_POINTS;
    int map_height;
    int map_width;
    std::vector<float> previous_location; // keep tracked of moved distance to limit backof
    std::vector<float> starting_area;
    float distance_moved_forwards; // meters moved since last time we moved backwards
    bool ok_to_back;
    bool done_exploring;
    bool is_backing;


public:
    ros::NodeHandle n;
    ros::Subscriber sub_obstacle;
    ros::Subscriber sub_object;
    ros::Subscriber sub_map;
    ros::Subscriber sub_IMU;
    ros::Subscriber sub_position;
    ros::Publisher pub_targets;
    ros::Publisher pub_exploration_done;
    ros::Publisher pub_speaker;

    Brain(ros::NodeHandle node)
    {
        n = node;
        stopped = false;
        abort1 = false;
        do_once = true;
        got_map = false;
        current_action_done = true;
        done_exploring = false;
        is_backing = false;

        N_FAILS = 0;
        MAX_FAILS = 15;
        sqrt_N_POINTS = 4;
        N_POINTS = pow(sqrt_N_POINTS,2);
        previous_location.resize(2);
        previous_location[0] = 0.0;
        previous_location[1] = 0.0;
        starting_area.resize(2);
        starting_area[0] = 0.23;
        starting_area[1] = 0.3;

        ok_to_back = 1;

        current_point_index = 0;
        exploration_targets.points.resize(pow(sqrt_N_POINTS,2));

        sub_obstacle = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &Brain::obstacleCallback,this);
        //sub_object = n.subscribe<geometry_msgs::PointStamped>("/found_object", 1, &Brain::objectCallback,this);
        sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/grid_map",1,&Brain::mapCallback,this);
        sub_position = n.subscribe<nav_msgs::Odometry>("/particle_position",1,&Brain::positionCallback,this);
        //sub_IMU = n.subscribe<sensor_msgs::Imu>("/imu/data",1,&Brain::imuCallback,this);

        pub_targets = n.advertise<sensor_msgs::PointCloud>("/explore_targets",1);
        pub_exploration_done = n.advertise<std_msgs::Bool>("/finished_exploring",1);
        pub_speaker = n.advertise<std_msgs::String>("/espeak/string",10);



        movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);
        gripper_client = n.serviceClient<arduino_servo_control::SetServoAngles>("/arduino_servo_control/set_servo_angles");
        sleep(0.5);
        gripperUp();
        sleep(0.5);
        gripperDown();
    }

    ~Brain()
    {

    }

    void backOff()
    {
        sound_msg.data = "Oops! Backing";
        pub_speaker.publish(sound_msg);
        N_FAILS++;
        ROS_INFO("BRAIN: Backing");
        //goal.final_point.position.x = 1;
        //goal.final_point.position.y = 1;
        //goal.final_point.orientation.z = 1;
        // add orientation

        //ROS_INFO("BRAIN: Waiting for server.");
        movement_client->waitForServer();
        // TODO wait Duration(x), otherwise restart server.

        //ROS_INFO("BRAIN: SERVER IS UP");
        goal.backwards = 1;
        goal.min_distance = 0.05;
        //ROS_INFO("BRAIN: Backwards = %s", goal.backwards ? "true" : "false" );
        movement_client->sendGoal(goal, boost::bind(&Brain::back_doneCb, this, _1, _2));
    }


    void explorationLoop()
    {
        //ROS_INFO("Point %i",current_point_index);
        if (!current_action_done)
        {
            return;
        }else if (done_exploring)
        {
          current_action_done = false;
          returnToStartingArea();
        }else if (current_point_index == N_POINTS)
        {
            // return to starting position
            ROS_INFO("Done exploring. Returning to starting area");
            sound_msg.data = "Woho! Done exploring. Returning to starting area";
            pub_speaker.publish(sound_msg);

            sleep(0.5);
            gripperUp();
            sleep(0.5);
            gripperDown();
            done_exploring = true;
            std_msgs::Bool done_msg;
            done_msg.data = 1;
            pub_exploration_done.publish(done_msg);
        }
        else if (N_FAILS > MAX_FAILS)
        {
            ROS_INFO("Gave up on point %i", current_point_index+1);
            sound_msg.data = "Point is impossible to reach! I give up.";
            pub_speaker.publish(sound_msg);

            exploration_targets.points[current_point_index].x = 0;
            exploration_targets.points[current_point_index].y = 0;
            current_point_index++;
            N_FAILS = 0;
            //movement_client->cancelGoal();
            //ROS_INFO("explorationLoop: too many fails. action done = true");
            current_action_done = true;
        }
        else
        {
            if (exploration_targets.points[current_point_index].x == 0 && exploration_targets.points[current_point_index].y == 0)
            {
              // point has already been passed by during exploration
              current_point_index++;
              return;
            }
            //ROS_INFO("explorationLoop: else case. action done = false");
            current_action_done = false;
            ROS_INFO("Moving to point %i",current_point_index+1);

            sound_msg.data = "Exploring point";
            pub_speaker.publish(sound_msg);

            explorePosition(exploration_targets.points[current_point_index].x, exploration_targets.points[current_point_index].y);
        }
    }

    void initExploration()
    {
        ROS_INFO("Init exploration.");
        if (!got_map)
        {
            ROS_INFO("No map..");
            return;
        }

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
                exploration_targets.points[k].x = i*map.info.resolution*height/(sqrt_N_POINTS+1) + (float) dis(gen);
                exploration_targets.points[k].y = j*map.info.resolution*width/(sqrt_N_POINTS+1) + (float) dis(gen);
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
        current_point_index = 0; // quick fix if positionCallback fucked shit up

        sound_msg.data = "Exploration initiated";
        pub_speaker.publish(sound_msg);
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
            goal.use_smooth_map = 0;
            ROS_INFO("BRAIN: Point %i explored. Moving on!", current_point_index+1);
            sound_msg.data = std::string("Point %i explored. Moving on!",current_point_index+1);
            pub_speaker.publish(sound_msg);
            sound_msg.data = "Easy";
            pub_speaker.publish(sound_msg);

            exploration_targets.points[current_point_index].x = 0;
            exploration_targets.points[current_point_index].y = 0;
            ROS_INFO("Succeded with point %i", current_point_index+1);
            current_point_index++;
            N_FAILS = 0;
            current_action_done = true;
        }else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            //No path found by server.
            N_FAILS++;
            if (goal.use_smooth_map == 1){
              moveToTemporaryGoal();
            }else{
            goal.use_smooth_map = 1;
            current_action_done =1;
          }
        }else if (!is_backing)
        {
            //N_FAILS++;
            //ROS_INFO("donecb: fail. action done = true");
            current_action_done = true;
        }
    }

    void temporaryGoalCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_movement::MovementResultConstPtr& result) {

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Succeeded temporary goal");
            goal.use_smooth_map = 0;
            current_action_done = true;
        }else
        {
           N_FAILS++;
           if (goal.use_smooth_map == 1)
           {
             goal.use_smooth_map = 0;
             moveToTemporaryGoal();
           }else if (goal.use_smooth_map == 0)
           {
             goal.use_smooth_map == 1;
             moveToTemporaryGoal();
           }
        }
    }

    void returnCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_movement::MovementResultConstPtr& result) {
        goal.use_smooth_map = 0;
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Succeded to reach stating area.");
            gripperUp();
            sleep(0.5);
            gripperDown();
            sleep(0.5);
            gripperUp();
            sleep(0.5);
            gripperDown();

            sound_msg.data = "Exploration over.";
            pub_speaker.publish(sound_msg);

            // braing loop ends here.
        }else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            //No path found by server.
            goal.use_smooth_map = 1;
            current_action_done = 1;


        }else if (!is_backing)
        {
            //N_FAILS++;
            //ROS_INFO("returncb: action done = true");
            current_action_done = true;
        }
    }

    void back_doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_movement::MovementResultConstPtr& result)
        {

        goal.use_smooth_map = 0;
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            //current_action_done = true;
            ROS_INFO("BRAIN: Backing succeded.");
            //N_FAILS--;
            //ROS_INFO("back_donecb: succeded. action done = true");
            current_action_done = true;

            //moveToPosition(goal.final_point.position.x, goal.final_point.position.y, goal.final_point.orientation.z);
            /*
            ROS_INFO("BRAIN: Point %i explored. Moving on!", current_point_index);
            exploration_targets.points[current_point_index].x = 0;
            exploration_targets.points[current_point_index].y = 0;
            current_point_index++;
            explorationLoop();
            */
        }else
        {
          ROS_INFO("BRAIN: backwards failed [%s]", state.toString().c_str());
          //ROS_INFO("back_donecb: fail. action done = true");
          current_action_done = true;
        }
        is_backing = false;
    }




    void moveToObject()
    {
        if (foundObjects.size() == 0)
        {
            ROS_INFO("BRAIN: No objects are found. Could not move to object position.");
            return;
        }

        movement_client->cancelGoal();

        //movement_server = new MovementAction("movement");
        //actionlib_movement::MovementGoal goal;

        goal.final_point.position.x = foundObjects[0].x;// x_dest;
        goal.final_point.position.y = foundObjects[0].y; //y_dest;
        goal.backwards = 0;
        goal.min_distance = 0.01;

        ROS_INFO("BRAIN: Waiting for server.");
        movement_client->waitForServer();
        // TODO wait Duration(x), otherwise restart server.

        ROS_INFO("BRAIN: SERVER IS UP");
        movement_client->sendGoal(goal, boost::bind(&Brain::doneCb, this, _1, _2));
    }

    void explorePosition(float x, float y)
    {
        //movement_server = new MovementAction("movement");
        //movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);

        movement_client->cancelGoal();

        goal.final_point.position.x = x;
        goal.final_point.position.y = y;
        goal.backwards = 0;
        goal.min_distance = 0.30;
        // add orientation

        //ROS_INFO("BRAIN: Waiting for server.");
        movement_client->waitForServer();
        // TODO wait Duration(x), otherwise restart server.

        //ROS_INFO("BRAIN: SERVER IS UP");
        movement_client->sendGoal(goal, boost::bind(&Brain::doneCb, this, _1, _2));
    }

    void moveToTemporaryGoal()
    {

        float x,y;
        int index;
        bool point_ok = 0;
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(-0.4, 0.4);

        while(!point_ok)
        {
            x = previous_location[0] + (float) dis(gen);
            y = previous_location[1] + (float) dis(gen);
            index = mToCell(y)*map_width+mToCell(x);
            if (index < map_width*map_height)
            {
              if(map.data[index] == 0)
              {
                  point_ok = 1;
              }
            }
        }
        goal.min_distance = 0.10;
        goal.final_point.position.x = x;
        goal.final_point.position.y = y;

        //ROS_INFO("BRAIN: SERVER IS UP");
        movement_client->sendGoal(goal, boost::bind(&Brain::temporaryGoalCb, this, _1, _2));
    }

    void returnToStartingArea()
    {
        //movement_server = new MovementAction("movement");
        //movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);

        //movement_client->cancelGoal();

        goal.final_point.position.x = starting_area[0];
        goal.final_point.position.y = starting_area[1];
        goal.backwards = 0;
        goal.min_distance = 0.02;
        // add orientation

        //ROS_INFO("BRAIN: Waiting for server.");
        movement_client->waitForServer();
        // TODO wait Duration(x), otherwise restart server.

        //ROS_INFO("BRAIN: SERVER IS UP");
        movement_client->sendGoal(goal, boost::bind(&Brain::returnCb, this, _1, _2));
    }


/*
    void objectCallback(const geometry_msgs::PointStamped::ConstPtr& msg){

      // NOT FUNCTIONAL, ADD MOVE TO POSITION FUNCTION!!!!!!!!!!!
      return
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
         //movement_client->cancelGoal();
         //ROS_INFO  ("BRAIN: The goal has been cancelled.");
         //moveToPosition(x,y,0);
     }

    }
    */

    void obstacleCallback(const std_msgs::Bool::ConstPtr& msg)
    {
       if (msg -> data == true && stopped == false)
       {
           stopped = true;
           is_backing = true;
           ROS_INFO  ("BRAIN: The goal has been cancelled. Mapping obstacle.");
           movement_client->cancelGoal();
           //ROS_INFO("ostaclecallback: . action done = false");
           current_action_done = false;
           sleep(3);
           //current_action_done = false;
           if (ok_to_back == 1)
           {
             is_backing = true;
             backOff();

           }else if (!is_backing)
           {
             //ROS_INFO("obstaclecallback: not backing. action done = true");
             current_action_done = true;
           }else
           {
             is_backing = false;
           }


       }else if (msg -> data == false && stopped == true)
       {
           ROS_INFO("BRAIN: Obstacle not visible anymore.");
           stopped = false;
       }
    }
/*
    void pauseCallback(const std_msgs::Bool::ConstPtr& msg)
    {
       if (msg -> data == true && stopped == false)
       {
           stopped = true;
           movement_client->cancelGoal();
           sleep(5);
           ROS_INFO  ("BRAIN: The goal has been cancelled. Evaluating object.");

       }else if (msg -> data == false && stopped == true)
       {
           ROS_INFO("BRAIN: Obstacle not visible anymore.");
           stopped = false;

           //explorationLoop();

       }
    }
*/
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        got_map = true;
        map = *map_msg;
        map_width = map_msg->info.width;
        map_height = map_msg->info.height;
        //ROS_INFO("got map res %f",map.info.resolution);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
      float acceleration_y = msg->linear_acceleration.y;

      if (abs(acceleration_y) > 3)
      {
          //stopped = true;
          movement_client->cancelGoal();
          ROS_INFO  ("IMU: we hit a wall!!!!");
          sound_msg.data = "Booooom!";
          pub_speaker.publish(sound_msg);
          is_backing = 1;
          N_FAILS++;
          backOff();
      }else
      {
          //stopped = false;
      }
    }

    void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      float dist; // = sqrt(pow(previous_location[0]-msg->pose.pose.position.x,2) + pow(previous_location[1]-msg->pose.pose.position.y,2));

      for (int i = current_point_index+1; i < N_POINTS; i++)
      {
        dist = sqrt(pow(exploration_targets.points[i].x-msg->pose.pose.position.x,2) + pow(exploration_targets.points[i].y-msg->pose.pose.position.y,2));
        if (dist < 0.15)
        {
          exploration_targets.points[i].x = 0;
          exploration_targets.points[i].y = 0;
          ROS_INFO("Passed by point %i. Removing point %i...",i+1,i+1);
        }
      }

      //ROS_INFO("Previous location: (%f, %f)", previous_location[0], previous_location[1]);
      //ROS_INFO("Current Location: (%f, %f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
      //ROS_INFO("A:%f",std::pow(previous_location[0]-msg->pose.pose.position.x,2));
      //ROS_INFO("B:%f",std::pow(previous_location[1]-msg->pose.pose.position.y,2));
      //float delta_x = (previous_location[0]-msg->pose.pose.position.x) * (previous_location[0]-msg->pose.pose.position.x) *100;
      //float delta_y = (previous_location[1]-msg->pose.pose.position.y) * (previous_location[1]-msg->pose.pose.position.y) *100;
      //ROS_INFO("%f",delta_x);
      //ROS_INFO("%f",delta_y);
      //dist = delta_x + delta_y;
      //printf("%f\n",dist);

      //dist = (previous_location[0]-msg->pose.pose.position.x) * (previous_location[0]-msg->pose.pose.position.x)  + (previous_location[1]-msg->pose.pose.position.y) * (previous_location[1]-msg->pose.pose.position.y);
      dist = sqrt(pow(previous_location[0]-msg->pose.pose.position.x,2) + pow(previous_location[1]-msg->pose.pose.position.y,2));
      float vel = (float) msg->twist.twist.linear.x;
      //ROS_INFO("Velocity: %f and Distance: %f", std::abs(vel), dist);
      if (std::abs(msg->twist.twist.linear.x) > 0.05 && dist < 0.015)
      {
        ROS_INFO("MOVING AGAINST A WALL!!!!!!!!!!!!!!!!!!");
        sound_msg.data = "Noooooooo. I am stuck";
        pub_speaker.publish(sound_msg);

        movement_client->cancelGoal();
        N_FAILS++;

        if (msg->twist.twist.linear.x < 0)
        {
          // backing against a wall. Replan path as usual
          current_action_done = 1;
        }else
        {
          // moving against a wall, back off
          //if (!is_backing)
          //{
            is_backing = 1;
            backOff();
          //}
        }

      }

      previous_location[0] = msg->pose.pose.position.x;
      previous_location[1] = msg->pose.pose.position.y;





      /*
      if (msg->twist.twist.linear.x < 0)
      {
        ok_to_back = 0;
        distance_moved_forwards = 0;
      }else if (dist > 0.03)
      {
        distance_moved_forwards += dist;
        previous_location[0] = msg->pose.pose.position.x;
        previous_location[1] = msg->pose.pose.position.x;

        for (int i = current_point_index+1; i < N_POINTS; i++)
        {
          dist = sqrt(pow(exploration_targets.points[i].x-msg->pose.pose.position.x,2) + pow(exploration_targets.points[i].y-msg->pose.pose.position.y,2));
          if (dist < 0.15)
          {
            exploration_targets.points[i].x = 0;
            exploration_targets.points[i].y = 0;
            ROS_INFO("Passed by point %i. Removing point %i...",i+1);
          }
        }
      }


      if (distance_moved_forwards > 0.2)
      {
        ok_to_back = 1;
      }
      */
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
    sleep(1);
  }

  return 0;
}
