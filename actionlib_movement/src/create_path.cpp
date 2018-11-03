#include <ros/ros.h>



class PathCreator
{
public:


        PathCreator()
        {

            point_distance = 0.01;

        }



        std::vector<float> getPath(float x0, float y0, float x1, float y1)
        {
            createLinearPath(x0,y0,x1,y1);
            return path;
        }




        void createLinearPath(float x0, float y0, float x1, float y1)
        {
            float phi = atan2(y1-y0,x1-x0);
            float dist = sqrt(pow(x0-x1,2) + pow(y0-y1,2));
            float current_dist = 0;

            float x = x0;
            float y = y0;

            while (current_dist <= dist)
            {
                path.push_back(x);
                path.push_back(y);
                x = x+point_distance*cos(phi);
                y = y+point_distance*sin(phi);
                current_dist = current_dist+point_distance;

            }
        }






private:
    std::vector<float> path;
    float x_start;
    float y_start;
    float x_end;
    float y_end;
    float point_distance;





};

/**
int main(int argc, char** argv){
        ros::init(argc, argv, "path_creator");
        ros::NodeHandle node("~");

        PathCreator my_path;
        std::vector<float> path_points;
        path_points = my_path.getPath(1,1,0,0);

        ROS_INFO("Path points:");
        for (int i = 0; i<path_points.size(); i = i+2)
        {
            ROS_INFO("[%f, %f]",path_points[i], path_points[i+1]);
        }



	return 0;
}
**/
