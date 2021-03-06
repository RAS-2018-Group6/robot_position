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
#include <random>
#include <string>
#include <cmath>
#include <object_identification/ObjectList.h>
#include <deque>
#include <map>


#define EXPLORE 1 // 1 for exploration mode, 0 for object retreival mode

class Brain
{
private:
    std::vector<ValuableObject> foundObjects;
    bool stopped;
    bool abort1;
    bool do_once;
    bool got_map;
    actionlib_movement::MovementGoal goal;
    actionlib::SimpleActionClient<actionlib_movement::MovementAction> *movement_client;
    ros::ServiceClient gripper_client;
    arduino_servo_control::SetServoAngles srv;
    sensor_msgs::PointCloud exploration_targets;
    sensor_msgs::PointCloud object_positions;
    nav_msgs::OccupancyGrid map;
    std_msgs::String sound_msg;
    object_identification::ObjectList known_objects_list;
    bool current_action_done;
    int current_point_index;
    int N_FAILS;
    int MAX_FAILS;
    int sqrt_N_POINTS;
    int N_POINTS;
    int map_height;
    int map_width;
    std::vector<float> previous_location; // keep tracked of moved distance to limit backof
    std::vector<float> current_location;	// to know what object to grab
    std::vector<float> starting_area;
    std::deque<float> stuck_velocity;
    std::deque<float> stuck_x;
    std::deque<float> stuck_y;
    float distance_moved_forwards; // meters moved since last time we moved backwards
    bool ok_to_back;
    bool done_exploring;
    bool is_backing;
    bool received_objects;

    // Additional variables used for object retrieval mode
    double object_position_x, object_position_y;
    int average_position_counter;
    bool retrieving_object;

    geometry_msgs::Twist vel;
    int current_object_index;
    int N_OBJECTS;
    bool going_to_object;
    bool carrying_object;
    bool catching_object;
    std::map<int,int> object_values;
    ros::Timer timer;


public:
    ros::NodeHandle n;
    ros::Subscriber sub_obstacle;
    ros::Subscriber sub_object;
    ros::Subscriber sub_position;
    ros::Publisher pub_targets;
    ros::Publisher pub_exploration_done;
    ros::Publisher pub_speaker;
    ros::Publisher pub_vel;

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
        sqrt_N_POINTS = 3; //3;
        N_POINTS = pow(sqrt_N_POINTS,2);

        stuck_velocity.resize(4);
        stuck_x.resize(4);
        stuck_y.resize(4);
        previous_location.resize(2);
        previous_location[0] = 0.23;
        previous_location[1] = 0.3;
        starting_area.resize(2);
        starting_area[0] = 0.23;
        starting_area[1] = 0.3;

        average_position_counter = 0;
        object_position_x = 0.0;
        object_position_y = 0.0;
        retrieving_object = false;

        current_point_index = 0;
        current_object_index = 0;
        N_OBJECTS = 0;

        goal.back_distance = 0.2;
        goal.forward_velocity = 0.15;

        ok_to_back = 1;



        sub_obstacle = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &Brain::obstacleCallback,this);
        sub_object = n.subscribe<geometry_msgs::PointStamped>("/found_object", 1, &Brain::objectCallback,this);
        sub_position = n.subscribe<nav_msgs::Odometry>("/particle_position",1,&Brain::positionCallback,this);

        pub_targets = n.advertise<sensor_msgs::PointCloud>("/explore_targets",1); // for visualization in rviz
        pub_exploration_done = n.advertise<std_msgs::Bool>("/finished_exploring",1);
        pub_speaker = n.advertise<std_msgs::String>("/espeak/string",10);
        pub_vel = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);



        movement_client = new actionlib::SimpleActionClient<actionlib_movement::MovementAction>("movement", true);
        gripper_client = n.serviceClient<arduino_servo_control::SetServoAngles>("/arduino_servo_control/set_servo_angles");

        if (EXPLORE == 1)
        {
            exploration_targets.points.resize(pow(sqrt_N_POINTS,2));
        }else{
            exploration_targets.points.resize(1);
            catching_object = false;
            going_to_object = true;
            carrying_object = false;
            gripperUp();
        }

    }

    void initData()
    {
        // get messages and set variables required before starting brain

        boost::shared_ptr<nav_msgs::OccupancyGrid const> msg;
        msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/grid_map",n);
        map = *msg; //ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/grid_map",n);
        got_map = true;
        map_width = map.info.width;
        map_height = map.info.height;

        if (EXPLORE == 0)
        {
            boost::shared_ptr<object_identification::ObjectList const> msg;
            msg = ros::topic::waitForMessage<object_identification::ObjectList>("/saved_objects",n);
            known_objects_list = *msg; //ros::topic::waitForMessage<object_identification::ObjectList>("/saved_objects",n);
            ROS_INFO("GOT THE SAVED OBJECTS");
            received_objects = true;
            N_OBJECTS = known_objects_list.positions.size();
            initObjectValues();
            setNewTargetObject();
            timer = n.createTimer(ros::Duration(180), &Brain::timerCallback,this);
        }else
        {
            timer = n.createTimer(ros::Duration(310), &Brain::timerCallback,this);
        }

    }

    ~Brain()
    {

    }

    void timerCallback(const ros::TimerEvent&)
    {
        // time is up. cancel
        if (!done_exploring)
        {
            movement_client->cancelGoal();
            done_exploring = true;
            sound_msg.data = "Time is up";
            pub_speaker.publish(sound_msg);
            std_msgs::Bool done_msg;
            done_msg.data = 1;
            if (EXPLORE == 1)
            {
                pub_exploration_done.publish(done_msg);
            }
        }

    }

    void backOff()
    {
        // command movement server to back off

        sound_msg.data = "Oops! Backing";
        pub_speaker.publish(sound_msg);
        N_FAILS++;

        if (catching_object == true && retrieving_object == false)
        {
            going_to_object = true;
        }
        catching_object = false;
        ROS_INFO("BRAIN: Backing");
        movement_client->waitForServer();
        goal.backwards = 1;
        goal.min_distance = 0.05;

        movement_client->sendGoal(goal, boost::bind(&Brain::back_doneCb, this, _1, _2));
    }

    void objectRetrievalLoop()
    {
        // loop running every spin during object retrieval

        if (!current_action_done)
        {
            return;
        }else if (done_exploring)
        {
            current_action_done = false;

        }else if (current_object_index == N_OBJECTS)
        {
            // return to starting position
            ROS_INFO("Done retrieving. Returning to starting area");
            sound_msg.data = "Woho! Done retreiving objects. Returning to starting area";
            pub_speaker.publish(sound_msg);
            current_action_done = false;
            done_exploring = true;
        }
        else if (N_FAILS > MAX_FAILS)
        {
            // give up and set new target object if failed to may times

            if(carrying_object == true){
                // never give up if currently carrying an object
                N_FAILS = 0;
                current_action_done = true;
            }
            else{
                ROS_INFO("Gave up on object %i", current_object_index+1);
                sound_msg.data = "Object is impossible to retrieve! I give up.";
                pub_speaker.publish(sound_msg);
                current_object_index++;
                setNewTargetObject();
                N_FAILS = 0;
                average_position_counter = 0;
                carrying_object = false;
                going_to_object = true;
                catching_object = false;
                current_action_done = true;
            }

        }
        else
        {
            // current action is done, send new target to movement server

            current_action_done = false;
            ROS_INFO("Moving to object %i",current_object_index+1);

            if(going_to_object){
                catching_object = false;
                sound_msg.data = "Retrieving object";
                pub_speaker.publish(sound_msg);
                gripperUp();
                moveToObject(exploration_targets.points[0].x, exploration_targets.points[0].y);
            }
            else if(carrying_object){
                goal.final_point.position.x = starting_area[0];
                goal.final_point.position.y = starting_area[1];
                goal.backwards = 0;
                goal.min_distance = 0.15;
                movement_client->waitForServer();
                movement_client->sendGoal(goal, boost::bind(&Brain::returnObjectCb, this, _1, _2));
            }
        }
    }

    void explorationLoop()
    {
        // loop that runs every spin during exploration

        if (!current_action_done)
        {
            return;
        }else if (done_exploring)
        {
            current_action_done = false;
            returnToStartingArea();
        }else if (current_point_index == N_POINTS)
        {
            // all points explored, return to starting position
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
            // could not reach point, move on to next point

            ROS_INFO("Gave up on point %i", current_point_index+1);
            sound_msg.data = "Point is impossible to reach! I give up.";
            pub_speaker.publish(sound_msg);

            exploration_targets.points[current_point_index].x = 0;
            exploration_targets.points[current_point_index].y = 0;
            current_point_index++;
            N_FAILS = 0;
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
            current_action_done = false;
            ROS_INFO("Moving to point %i",current_point_index+1);

            sound_msg.data = "Exploring point";
            pub_speaker.publish(sound_msg);

            explorePosition(exploration_targets.points[current_point_index].x, exploration_targets.points[current_point_index].y);
        }
    }

    void initExploration()
    {
        // generate valid exploration points

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
                exploration_targets.points[k].x = i*map.info.resolution*width/(sqrt_N_POINTS+1) + (float) dis(gen);
                exploration_targets.points[k].y = j*map.info.resolution*height/(sqrt_N_POINTS+1) + (float) dis(gen);
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
        // callback for movement to exploration target

        ROS_INFO("BRAIN: server responded with state [%s]", state.toString().c_str());

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            goal.use_smooth_map = 0;
            ROS_INFO("BRAIN: Point %i explored. Moving on!", current_point_index+1);
            sound_msg.data = std::string("Point explored. Moving on!",current_point_index+1);
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
            current_action_done = true;
        }
    }

    void temporaryGoalCb(const actionlib::SimpleClientGoalState& state,
                         const actionlib_movement::MovementResultConstPtr& result) {
        // callback for movement to temporary goal

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
        // callback for returning to starting area after exploration is done

        goal.use_smooth_map = 0;
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Succeded to reach starting area.");

            sound_msg.data = "Exploration over.";
            pub_speaker.publish(sound_msg);

            // brain loop ends here.
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

    void returnObjectCb(const actionlib::SimpleClientGoalState& state,
                        const actionlib_movement::MovementResultConstPtr& result) {
        // callback to return object to starting area

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Succeded to reach stating area.");
            gripperUp();
            setNewTargetObject();

            average_position_counter = 0;
            object_position_x = 0.0;
            object_position_y = 0.0;
            goal.back_distance = 0.8;
            goal.use_smooth_map = 0;
            ROS_INFO("Succeded with object %i", current_object_index+1);
            current_object_index++;

            N_FAILS = 0;


            sound_msg.data = "I am the best";
            pub_speaker.publish(sound_msg);

            backOff();

            carrying_object = false;
            going_to_object = true;
            retrieving_object = false;

        }else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            //No path found by server.
            goal.use_smooth_map = 1;
            current_action_done = 1;


        }else if (!is_backing)
        {
            current_action_done = true;
        }
    }

    void back_doneCb(const actionlib::SimpleClientGoalState& state,
                     const actionlib_movement::MovementResultConstPtr& result)
    {
        // callback for backwards movement

        goal.use_smooth_map = 0;
        goal.back_distance = 0.2;
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {

            ROS_INFO("BRAIN: Backing succeded.");
            current_action_done = true; // loop will restart the previous goal

        }else
        {
            ROS_INFO("BRAIN: backwards failed [%s]", state.toString().c_str());
            //ROS_INFO("back_donecb: fail. action done = true");
            current_action_done = true;
        }
        is_backing = false;
    }


    void catchingCb(const actionlib::SimpleClientGoalState& state,
                    const actionlib_movement::MovementResultConstPtr& result) {
        // callback for catching and object

        ROS_INFO("BRAIN: server responded with state [%s]", state.toString().c_str());

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            gripperDown();
            goal.use_smooth_map = 0;
            retrieving_object = false;
            carrying_object = true;
            going_to_object = false;
            goal.forward_velocity = 0.15;
            current_action_done = true;
        }else
        {
            goal.use_smooth_map = 1;
            movement_client->sendGoal(goal, boost::bind(&Brain::catchingCb, this, _1, _2));
        }
    }

    void doneObjectCb(const actionlib::SimpleClientGoalState& state,
                      const actionlib_movement::MovementResultConstPtr& result) {
        // callback for moving to object position, if succeeded, the robot reached the object position without seeing the object
        ROS_INFO("BRAIN: server responded with state [%s]", state.toString().c_str());

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            goal.forward_velocity = 0.15;
            goal.use_smooth_map = 0;
            ROS_INFO("BRAIN: Object reached.");
            current_object_index++;

            setNewTargetObject();

            retrieving_object = false;
            carrying_object = false;
            going_to_object = true;

            current_action_done = 1;

        }else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            //No path found by server.
            N_FAILS++;
            if (goal.use_smooth_map == 0){
                goal.use_smooth_map = 1;
                current_action_done =1;
            }
        }else if (!is_backing)
        {
            current_action_done = true;
        }
    }

    void moveToObject(float x, float y)
    {
        // move to object position

        goal.final_point.position.x = x;
        goal.final_point.position.y = y;
        goal.backwards = 0;
        goal.min_distance = 0.05; // required distance for position to count as reached

        movement_client->sendGoal(goal, boost::bind(&Brain::doneObjectCb, this, _1, _2));
    }

    void explorePosition(float x, float y)
    {
        // move to exploration point

        movement_client->cancelGoal();

        goal.final_point.position.x = x;
        goal.final_point.position.y = y;
        goal.backwards = 0;
        goal.min_distance = 0.30;
        movement_client->waitForServer();
        movement_client->sendGoal(goal, boost::bind(&Brain::doneCb, this, _1, _2));
    }

    void moveToTemporaryGoal()
    {
        // used in emergency cases if the robot got stuck in a narrow area and cant get out after several attempts. Generates a random goal point close to the robot.

        float x,y;
        int index;
        bool point_ok = 0;
        std::random_device rd;
        std::mt19937 gen(rd());
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

        movement_client->sendGoal(goal, boost::bind(&Brain::temporaryGoalCb, this, _1, _2));
    }

    void returnToStartingArea()
    {
        goal.final_point.position.x = starting_area[0];
        goal.final_point.position.y = starting_area[1];
        goal.backwards = 0;
        goal.min_distance = 0.02;
        movement_client->sendGoal(goal, boost::bind(&Brain::returnCb, this, _1, _2));
    }


    void objectCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
        // called when perception sees a valuable object

        if(!catching_object){
            ROS_INFO("Not catching");
            if(retrieving_object){
                ROS_INFO("Allowed to retrieve");
                going_to_object = false;
                current_action_done = false;
                if(average_position_counter == 0){
                    movement_client->cancelGoal();
                    object_position_x = msg->point.x;
                    object_position_y = msg->point.y;
                    average_position_counter++;
                    ROS_INFO("Object found. Estimating position for retrieval");
                }
                else{
                    object_position_x = msg->point.x;
                    object_position_y = msg->point.y;
                    average_position_counter++;
                }
                ROS_INFO ("Detected object position: (%f,%f)",object_position_x,object_position_y);
                if(average_position_counter > 5){
                    catching_object = true;
                    goal.final_point.position.x = object_position_x;
                    goal.final_point.position.y = object_position_y;
                    goal.backwards = 0;
                    goal.min_distance = 0.03;
                    movement_client->sendGoal(goal, boost::bind(&Brain::catchingCb, this, _1, _2));

                }
            }
        }

    }

    void obstacleCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        // called when perception sees an obstacle in front of the robot

        if (msg -> data == true && stopped == false && catching_object == false)
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


        }else if (msg -> data == false && stopped == true && catching_object == false)
        {
            ROS_INFO("BRAIN: Obstacle not visible anymore.");
            stopped = false;
        }
    }


    void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // estimate of current position from the particle filter

        float dist;

        if (EXPLORE == 1){
            for (int i = current_point_index+1; i < N_POINTS; i++)
            {
                // mark future exploration point as done if passed by
                dist = sqrt(pow(exploration_targets.points[i].x-msg->pose.pose.position.x,2) + pow(exploration_targets.points[i].y-msg->pose.pose.position.y,2));
                if (dist < 0.15)
                {
                    exploration_targets.points[i].x = 0;
                    exploration_targets.points[i].y = 0;
                    ROS_INFO("Passed by point %i. Removing point %i...",i+1,i+1);
                }
            }
        }

        dist = sqrt(pow(previous_location[0]-msg->pose.pose.position.x,2) + pow(previous_location[1]-msg->pose.pose.position.y,2));
        float vel = (float) msg->twist.twist.linear.x;


        float mean_v = stuck_velocity[0];
        float tot_dist = 0;
        for (int i = 1; i < stuck_x.size(); i++)
        {
            mean_v += stuck_velocity[i];
            tot_dist += sqrt(pow(stuck_x[i-1]-stuck_x[i],2) + pow(stuck_y[i-1]-stuck_y[i],2));
        }
        mean_v = (mean_v+std::abs(msg->twist.twist.linear.x))/(stuck_velocity.size()+1);
        tot_dist += sqrt(pow(stuck_x[stuck_x.size()-1]-msg->pose.pose.position.x,2) + pow(stuck_y[stuck_y.size()-1]-msg->pose.pose.position.y,2));
        stuck_x.pop_front();
        stuck_y.pop_front();
        stuck_x.push_back(msg->pose.pose.position.x);
        stuck_y.push_back(msg->pose.pose.position.y);
        stuck_velocity.pop_front();
        stuck_velocity.push_back(msg->twist.twist.linear.x);

        // same position but non-zero velocity over several callbacks, robot must be stuck
        if (std::abs(mean_v) > 0.04 && tot_dist < 0.015)
        {
            ROS_INFO("MOVING AGAINST A WALL!!!!!!!!!!!!!!!!!!");
            sound_msg.data = "Noooooooo. I am stuck";
            pub_speaker.publish(sound_msg);

            movement_client->cancelGoal();


            if (msg->twist.twist.linear.x < 0)
            {
                // backing against a wall. Replan path as usual with forward movement
                N_FAILS++;
                current_action_done = 1;
            }else
            {
                is_backing = 1;
                backOff();
            }

        }

        if(is_backing == false && EXPLORE == 0 && !carrying_object) {
            // if close to object position, decrease speed to easier see object

            float dist_to_goal;
            dist_to_goal = sqrt(pow(msg->pose.pose.position.x-exploration_targets.points[0].x,2) + pow(msg->pose.pose.position.y-exploration_targets.points[0].y,2));
            if (dist_to_goal < 0.5){

                if (!retrieving_object)
                {
                    ROS_INFO("Close to object");
                    sound_msg.data = "Im gonna get ya";
                    pub_speaker.publish(sound_msg);
                    retrieving_object = true;
                    goal.forward_velocity = 0.08;
                    movement_client->cancelGoal();
                    current_action_done = true;
                }
            }
            else{
                goal.forward_velocity = 0.15;
                if(carrying_object == false){
                    going_to_object = true;
                }
                retrieving_object = false;
            }


        }
        previous_location[0] = msg->pose.pose.position.x;
        previous_location[1] = msg->pose.pose.position.y;
    }


    void setNewTargetObject()
    {
        // set new target object

        ROS_INFO("SETTING NEW TARGET OBJECT");
        float min_x,min_y,min_ind,min_dist,dist;
        bool sort_by_value = 0;

        // some high value
        min_x = 10;
        min_y = 10;
        min_dist = 10;

        for(int i = 0; i < N_OBJECTS; i++)
        {
            if (sort_by_value)
            {
                dist = object_values[known_objects_list.object_class[i]]; // value
                if (dist > min_dist)
                {
                    // min dist == max value in this case
                    min_x = known_objects_list.positions[i].point.x;
                    min_y = known_objects_list.positions[i].point.y;
                    min_dist = dist;
                    min_ind = i;
                }
            }else
            {
                dist = sqrt(pow(known_objects_list.positions[i].point.x-previous_location[0],2)+pow(known_objects_list.positions[i].point.y-previous_location[1],2));
                if (dist < min_dist)
                {
                    min_x = known_objects_list.positions[i].point.x;
                    min_y = known_objects_list.positions[i].point.y;
                    min_dist = dist;
                    min_ind = i;
                }
            }
        }
        ROS_INFO("Picked object %i", known_objects_list.id[min_ind]);

        //set target to closest point
        exploration_targets.points[0].x = min_x;
        exploration_targets.points[0].y = min_y;

        // "remove" closest point from msg
        known_objects_list.positions[min_ind].point.x = 100;
        known_objects_list.positions[min_ind].point.y = 100;
        known_objects_list.object_class[min_ind] = 9; // set to low valued class so not picked again

    }

    void initObjectValues()
    {
        // object values for the competition

        object_values[0] = 10000; //yellow ball
        object_values[1] = 1000; //yellow cube
        object_values[2] = 1000; //green cube
        object_values[3] = 1000; //green cylinder
        object_values[4] = 100; //green hollow cube
        object_values[5] = 100; //orange cross
        object_values[6] = 5000; //patric
        object_values[7] = 1000; //red cylinder
        object_values[8] = 100; //red hollow cube
        object_values[9] = 10000; //red ball
        object_values[10] = 1000; //blue cube
        object_values[11] = 5000; //blue triangle
        object_values[12] = 100; //purple cross
        object_values[13] = 5000; //purple star
        object_values[14] = 0; //purple star
    }


};



int main (int argc, char **argv)
{
    ros::init(argc, argv, "exploration_client");
    ros::NodeHandle n;

    Brain brain(n);
    ROS_INFO("Starting Brain.");

    brain.initData();

    if (EXPLORE == 1){
        brain.initExploration();
        while(ros::ok())
        {
            brain.publishPoints();
            brain.explorationLoop();
            ros::spinOnce();
            sleep(0.5);
        }
    }
    else{
        while(ros::ok())
        {
            brain.objectRetrievalLoop();
            brain.publishPoints();
            ros::spinOnce();
            sleep(0.5);
        }
    }

    return 0;
}
