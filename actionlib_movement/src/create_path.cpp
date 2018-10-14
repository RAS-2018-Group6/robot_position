#include <ros/ros.h>



class PathCreator
{
public:


        PathCreator()
        {

            point_distance = 0.1;

        }



        std::vector<double> getPath(double x0, double y0, double x1, double y1)
        {
            createLinearPath(x0,y0,x1,y1);
            return path;
        }




        void createLinearPath(double x0, double y0, double x1, double y1)
        {
            double phi = atan2(y1-y0,x1-x0);
            double dist = sqrt(pow(x0-x1,2) + pow(y0-y1,2));
            double current_dist = 0;

            double x = x0;
            double y = y0;

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
    std::vector<double> path;
    double x_start;
    double y_start;
    double x_end;
    double y_end;
    double point_distance;





};

/**
int main(int argc, char** argv){
        ros::init(argc, argv, "path_creator");
        ros::NodeHandle node("~");

        PathCreator my_path;
        std::vector<double> path_points;
        path_points = my_path.getPath(1,1,0,0);

        ROS_INFO("Path points:");
        for (int i = 0; i<path_points.size(); i = i+2)
        {
            ROS_INFO("[%f, %f]",path_points[i], path_points[i+1]);
        }



	return 0;
}
**/
