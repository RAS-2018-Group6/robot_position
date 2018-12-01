#include <stdio.h>
#include <ros/ros.h>
//#include <std_msgs/UInt32>
//#include <std_msgs/Float32>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include "class_cell.hpp"
#include <iostream>
#include <fstream>
//#include "/home/ras16/catkin_ws/src/robot_map/src/map_node.cpp"


#define EMPTY -1
#define VALUE 1
//#define HIGH_COST 10
#define FREE 30 //Asume there is nothing if the value is less than FREE
#define UNKNOWN -7
#define WALL 98
#define THICK 50
#define ROBSIZE 0.13 //m (size of the robot that we want to thicken the walls with)

/*class MapNode{
	public:
		int nRows = 20;
		int nColumns = 20;
		int map_resolution = 20;

		int mToCell(double x){return round(x/map_resolution);}
		std::vector<std::vector<int> > coords;

};*/

class PathCreator{

	private:
		//MapNode map;
		int nRows, nColumns;
		float map_resolution;
		std::vector<signed char> data;
		std::vector<std::vector<signed char> > map;
		int ROBSIZE_Cell;



	public:

		//constructor
		PathCreator(int height) {map.resize(height);}
		cell new_cell(int coords[2], double f, double g){

			cell newcell;
			newcell.coords[0]=coords[0];
			newcell.coords[1]=coords[1];
			newcell.f=f;
			newcell.g=g;
			newcell.parent_coords[0]=EMPTY; //I fill this in later
			newcell.parent_coords[1]=EMPTY; //I fill this in later

			return newcell;

		}

		//Print map to disk
		void printMap(std::vector<cell> path){
			char output [nColumns][nRows];
			for(int i=0;i<nColumns;i++){
				for(int j=0;j<nRows;j++){
					if(map[i][j] < THICK){
						output[i][j] = '.';
					}
					else if(map[i][j]==THICK){
						output[i][j] = '/';
					}
					else if(map[i][j]>WALL){
						output[i][j] = '#';
					}

				}
			}
			std::ofstream myfile;
			myfile.open("/home/ras16/paths/map_visualization.txt");
			for(int i=0;i<nColumns;i++){
				for(int j=0;j<nRows;j++){
					myfile << output[i][j];
				}
				myfile << "\n";
			}
			myfile.close();
			int path_size = path.size();
			for(int k=0;k<path_size;k++){
					int i = path[k].coords[0]; // x
					int j = path[k].coords[1]; // y
					output[i][j] = 'x';
			}
			std::ofstream myfile_2;
			myfile_2.open("/home/ras16/paths/path_visualization.txt");
			for(int i=0;i<nColumns;i++){
				for(int j=0;j<nRows;j++){
					myfile_2 << output[i][j];
				}
				myfile_2 << "\n";
			}
			myfile_2.close();
		}

//This function is already in Kristian's code
		int mToCell(float x)
    {
        return (int) round(x/map_resolution);
    }

		void mapMatrix(){ //Converts the data into a matrix
			//ROS_INFO("Transform to Map array");
			for (int i =0; i<nColumns; i++){
				//ROS_INFO("%i",i);
				map[i].resize(nRows, UNKNOWN); //Give the vectors of the matrix a length
			}
			//ROS_INFO("TEst");
			for(int i = 0; i<nColumns; i++){
				for (int j = 0; j < nRows; j++){
					map[i][j]=data[j*nColumns+i];
				}
			}
		}

		void smoothMap(){
			for (int i = 0; i < nColumns; i++){
				for (int j = 0; j<nRows; j++){
					//ROS_INFO("nEXT");
					if (map[i][j]>WALL){
						//Make the wall thicker (put a value <WALL and >FREE) in a squared area of size 2*ROBSIZE
						for (int m = -ROBSIZE_Cell; m <= ROBSIZE_Cell; m++){
							for (int n = -ROBSIZE_Cell; n <= ROBSIZE_Cell; n++){
								if (i>=(-m) && (i+m)<(nColumns) && j>=(-n) && (j+n)<(nRows) && map[i+m][j+n]<WALL){
									map[i+m][j+n]=THICK;
								}
							}
						}
					}
					//ROS_INFO("Out of if, i = %i,  j= %i ", i, j);
				}
			}
			//ROS_INFO("Finished wall thickening");
		}

		void unsmoothPoints(int x, int y){
			for (int m = -(ROBSIZE_Cell-1); m <= (ROBSIZE_Cell-1); m++){
				for (int n = -(ROBSIZE_Cell-1); n <= (ROBSIZE_Cell-1); n++){
					if (x>=(-m) && (x+m)<(nColumns) && y>=(-n) && (y+n)<(nRows) && map[x+m][y+n]<WALL){
						map[x+m][y+n]=0; //Clear the points next to the given point
					}
				}
			}

		//ROS_INFO("Finished clearing %i, %i", x, y);
		}

		bool free_line(int x0, int y0, int x1, int y1){
			bool free = true; //The line doesn't cross walls when free is true
			float x_m0, y_m0, x_m1, y_m1;
			std::vector<float> line (0);
			x_m0 = x0*map_resolution;
			y_m0 = y0*map_resolution;
			x_m1 = x1*map_resolution;
			y_m1 = y1*map_resolution;
			float dist = sqrt(pow(x_m0-x_m1,2) + pow(y_m0-y_m1,2));
			float current_dist = 0;
			float phi = atan2 ((y_m1-y_m0), (x_m1-x_m0));
			int x_check, y_check;
			float point_distance = 0.01;

			float x = x_m0;
      float y = y_m0;
			//ROS_INFO("is line from cell %i, %i to cell %i, %i free?:", x0, y0, x1, y1);

			while(current_dist <= dist){
				line.push_back(x);
        line.push_back(y);
        x = x+point_distance*cos(phi);
        y = y+point_distance*sin(phi);
        current_dist = current_dist+point_distance;
			}

			for (int i = 0; i<line.size(); i+=2){
				x_check = mToCell(line[i]);
				y_check = mToCell(line[i+1]);
				if (map[x_check][y_check] > FREE){ //The cell is occupied
	//				ROS_INFO("The cell is occupied");
					return false; //If there is a point in the line occupied by a wall, then the path is not free
				}
				//ROS_INFO("The cell is free");
			}
			//ROS_INFO("Free line");

			return true;
		}

		std::vector<cell> smoothPath(std::vector<cell> path){
			std::vector<cell> smooth_path;
			std::vector<cell>::iterator c = path.begin()+2;
			int iterations = 2;
			smooth_path.push_back(path[0]);
			smooth_path.push_back(path[1]);
			while (c <= path.end() && iterations < path.size()){ //both should be the same, but just in case
				for (int j = 1; j < path.size()-1; j++){
					while (free_line(smooth_path[smooth_path.size()-2].coords[0],smooth_path[smooth_path.size()-2].coords[1],path[iterations].coords[0],path[iterations].coords[1]) && path.size()>0){
						//As long as the line between the two points is free
					//	ROS_INFO("1. iterations = %i, path size = %i", iterations, path.size());

						smooth_path.pop_back();
				//		ROS_INFO("2. iterations = %i, path size = %i", iterations, path.size());

						smooth_path.push_back(path[iterations]);
			//			ROS_INFO("3. iterations = %i, path size = %i", iterations, path.size());

						if (c<= path.end()){
						//	ROS_INFO("4.1 iterations = %i, path size = %i", iterations, path.size());

							path.erase(c);
							//ROS_INFO("4. iterations = %i, path size = %i", iterations, path.size());
						}
						//else ROS_INFO("4. Not erased");
						if (c== path.end()){
							//ROS_INFO("4. FINISHED PATH iterations = %i, path size = %i", iterations, path.size());
							break;
						}

					}
					smooth_path.push_back(path[iterations]);
					if (c< path.end()){

						path.erase(c);
						c++;

			//			ROS_INFO("Continue", iterations, path.size());
					}
					iterations++;
				//	ROS_INFO("New Point Added to smooth path");
				}
				smooth_path.push_back(path[iterations-1]);
			}
			//ROS_INFO("Number of points of the smooth path: %i", smooth_path.size());

			//ROS_INFO("The path has been smoothed");

			return smooth_path;
		}

		double heuristic(int coords[2], int goal_coords[2]){
			double heuristic = 0.0;
			if (goal_coords[0] != EMPTY && goal_coords[1] != EMPTY){
				heuristic = std::abs(coords[0]-goal_coords[0]) + std::abs (coords[1]-goal_coords[1]);
				//Variation of the algorithm,  to make it faster: multiply heuristic *VALUE
				heuristic *= VALUE;
			}
			return heuristic;
		}

		//Maybe the g cost can always be 1, not taking into consideration possible rotations cause a higher cost

		double g_cost(int coords[2], int coords_before[2], int parents_coords[2]){
			/*if (coords_before [0] - parents_coords[0] != 0){ //movement along x axis
				if (coords[0]-coords_before[0] != 0) return 1 //The direction of the movement is the same
				else return HIGH_COST; //Cost of rotation
			}
			else{ //movement along y axis
				if (coords[1]-coords_before[1] != 0) return 1 //The direction of the movement is the same
				else return HIGH_COST; //Cost of rotation
			}*/
			return 1;
		}

		std::vector<cell> build_path(cell goal, set_of_cells * close){

			cell now=goal;
			std::vector<cell> path_vector(1,goal);

			while(now.parent_coords[0]!=EMPTY && now.parent_coords[1] != EMPTY){//While the cell we are looking at had been obtained from another one
			//(was not the first one)

				now=(*close).pop(now.parent_coords); //Save in "now" the cell that was the parent of the one I was in

				path_vector.push_back(now);

			}
									//ROS_INFO("End of build path");
				return path_vector;
		}


		std::vector<cell> astar(int x_robot, int y_robot, int x_goal, int y_goal){

			int lengthx = nRows;
			int lengthy = nColumns; //Number of cells in each direction that the map has
			int goal_coords[2]={x_goal,y_goal},coords[2],before[2],cost=0;
			double g, f, h;


			std::vector<cell> not_valid;
			mapMatrix();
			ROBSIZE_Cell = mToCell (ROBSIZE);
			smoothMap();
			unsmoothPoints(x_robot, y_robot);
			unsmoothPoints(x_goal, y_goal);
			//ROS_INFO("Start A*");

			g = 0;
			h = heuristic(coords, goal_coords);
			f = g+h;






			cell now,trial;

			coords[0]=x_robot;
			coords[1]=y_robot;
			before[0]=EMPTY;
			before[1]=EMPTY;

			now = new_cell (coords, f, g);

			//Create the lists of points to accumulate checkpoints

			set_of_cells close; //In the beginning it must be empty
			set_of_cells open(now); //The first element must be the first cell to check (the one in which the robot is)
			set_of_cells before_list; //The list of cells that can still be checked



			//ROS_INFO("now coord = %i, %i", now.coords[0], now.coords[1]);

			not_valid.push_back(now); //Put in the not valid vector the values of the current position

			//First, we check that the goal coordinates are within the limits of the map:

			if(x_goal < 0 || y_goal < 0 || x_goal >= lengthx || y_goal >= lengthy){

				//ROS_INFO("The coordinates are not within the limits");

				return not_valid;
			}


int iter = 0;

			while(!open.empty()) {//While there are still cells to check in the map
				//ROS_INFO("iteration %i", iter);
				iter++;
			//	ROS_INFO("open: x = %i y =  %i", open.cell_array[0].coords[0],open.cell_array[0].coords[1] );
				now=open.best_cell(); //save in now the cell with the lowest cost function (f) of the open list
//ROS_INFO("Open has something: %i",!open.empty());

				if(now.coords[0]==goal_coords[0] && now.coords[1]==goal_coords[1]){ //If I already have the solution for the path:
					//ROS_INFO("Path is built. goal is %i, %i", goal_coords[0],goal_coords[1]);
					return build_path(now, &close); //Return the path (calling the function build_path)


				}

				//If I haven't found the path yet, try the cells next to the cell now:
				close.add(now); //Add the cell I'm in to the list

				if(now.coords[1]-1>=0){ //If I'm not in the limit 0 of y:
					coords[0]=now.coords[0]; //try new coordinates
					coords[1]=now.coords[1]-1;
					//ROS_INFO("3 x_coord = %i , y_coord = %i, map = %i", coords[0], coords[1], map[coords[0]][coords[1]]);

					if(map[coords[0]][coords[1]] <= FREE){ //If that cell is free of obstacles
						//ROS_INFO("inside map: x_coord = %i , y_coord = %i", coords[0], coords[1]);
						h=heuristic(trial.coords,goal_coords); //compute heuristic
						g = now.g + g_cost(coords,now.coords,now.parent_coords); //update g cost
						f = g + h; // compute total cost

						if(open.inside(coords)){
							trial = open.get(coords);
						}
						else{
							trial = new_cell(coords,f,g);
						}

						if(!close.inside(trial.coords) || f < trial.f ){ //If that cell is not in the list still OR it has higher value f
							trial.g=g; //update the values of that cell
							trial.f=f;
							trial.parent_coords[0]=now.coords[0];
							trial.parent_coords[1]=now.coords[1];
							if(!open.inside(trial.coords)) //If that cell is not in this list
								open.add(trial); //include it
							else{ // update the value of the cell in open
								open.pop(trial.coords);
								open.add(trial);
							}
						}
					}
				}
				//ROS_INFO("aFTER IF NUMBER 3");
				if(now.coords[1]+1<lengthy){ //If I'm not in the limit length of y:
					coords[0]=now.coords[0]; //try new coordinates
					coords[1]=now.coords[1]+1;
					//ROS_INFO("4 x_coord = %i , y_coord = %i, map = %i", coords[0], coords[1], map[coords[0]][coords[1]]);

					if(map[coords[0]][coords[1]]  <= FREE){ //If that cell is free of obstacles
					//	ROS_INFO("inside map: x_coord = %i , y_coord = %i", coords[0], coords[1]);
						h=heuristic(trial.coords,goal_coords); //compute heuristic
						g = now.g + g_cost(coords,now.coords,now.parent_coords); //update g cost
						f = g + h; // compute total cost

						if(open.inside(coords)){
							trial = open.get(coords);
						}
						else{
							trial = new_cell(coords,f,g);
						}

						if(!close.inside(trial.coords) || f < trial.f ){ //If that cell is not in the list still OR it has higher value f
							trial.g=g; //update the values of that cell
							trial.f=f;
							trial.parent_coords[0]=now.coords[0];
							trial.parent_coords[1]=now.coords[1];
							if(!open.inside(trial.coords)) //If that cell is not in this list
								open.add(trial); //include it
							else{ // update the value of the cell in open
								open.pop(trial.coords);
								open.add(trial);
							}
						}
					}
				}

			//	ROS_INFO("now coord = %i, %i", now.coords[0], now.coords[1]);

				if(now.coords[0]-1>=0){ //If I'm not in the limit 0 of x:
					coords[0]=now.coords[0]-1; //try new coordinates
					coords[1]=now.coords[1];
			//	ROS_INFO("1 x_coord = %i , y_coord = %i, map = %i", coords[0], coords[1], map[coords[0]][coords[1]]);


					if(map[coords[0]][coords[1]] <= FREE){//If that cell is free of obstacles
					//	ROS_INFO("inside map: x_coord = %i , y_coord = %i", coords[0], coords[1]);
						h=heuristic(trial.coords,goal_coords); //compute heuristic
						g = now.g + g_cost(coords,now.coords,now.parent_coords); //update g cost
						f = g + h; // compute total cost

						if(open.inside(coords)){
							trial = open.get(coords);
						}
						else{
							trial = new_cell(coords,f,g);
						}

						if(!close.inside(trial.coords) || f < trial.f ){ //If that cell is not in the list still OR it has higher value f
							trial.g=g; //update the values of that cell
							trial.f=f;
							trial.parent_coords[0]=now.coords[0];
							trial.parent_coords[1]=now.coords[1];
							if(!open.inside(trial.coords)) //If that cell is not in this list
								open.add(trial); //include it
							else{ // update the value of the cell in open
								open.pop(trial.coords);
								open.add(trial);
							}
						}
					}
				}
			//	ROS_INFO("aFTER IF NUMBER 1");
			//	ROS_INFO("NOW COORDS X = %i, lengthx = %i", now.coords[0], lengthx);

				if(now.coords[0]+1<lengthx){ //If I'm not in the limit legth of x:

					coords[0]=now.coords[0]+1; //try new coordinates

					coords[1]=now.coords[1];
				//	ROS_INFO("2 x_coord = %i , y_coord = %i, map = %i", coords[0], coords[1], map[coords[0]][coords[1]]);
				//	ROS_INFO("hELOOOO");
					if(map[coords[0]][coords[1]] <= FREE){ //If that cell is free of obstacles
					//	ROS_INFO("inside map: x_coord = %i , y_coord = %i", coords[0], coords[1]);
						h=heuristic(trial.coords,goal_coords); //compute heuristic
						g = now.g + g_cost(coords,now.coords,now.parent_coords); //update g cost
						f = g + h; // compute total cost

						if(open.inside(coords)){
							trial = open.get(coords);
						}
						else{
							trial = new_cell(coords,f,g);
						}

						if(!close.inside(trial.coords) || f < trial.f ){ //If that cell is not in the list still OR it has higher value f
							trial.g=g; //update the values of that cell
							trial.f=f;
							trial.parent_coords[0]=now.coords[0];
							trial.parent_coords[1]=now.coords[1];
							if(!open.inside(trial.coords)) //If that cell is not in this list
								open.add(trial); //include it
							else{ // update the value of the cell in open
								open.pop(trial.coords);
								open.add(trial);
							}
						}
					}
				}
			//	ROS_INFO("aFTER IF NUMBER 2");


			//	ROS_INFO("aFTER IF NUMBER 4");
			}
			//ROS_INFO("No path has been found");

			return not_valid; //If we are here, it means that there is no path to that point (the while loop has finished and no path has been found)
		}

		std::vector<float> getPath (float x_robot, float y_robot, float x_dest, float y_dest, int rows, int columns, float resolution,std::vector<signed char> map_data){
			//Convert coordinates into cells and send to astar:
			std::vector<cell> path_cell;

			nRows = rows;
			nColumns = columns;
			map_resolution = resolution;
			data = map_data;

			path_cell = astar(mToCell(x_robot),mToCell(y_robot),mToCell(x_dest),mToCell(y_dest));
			//ROS_INFO("number of path points: %i",path_cell.size());

			if (path_cell.size()>1){ //Call the smoothing function just if the path exists
				path_cell = smoothPath(path_cell);
			}
			int size_path = path_cell.size()*2;
			std::vector<float> path (size_path);

			//ROS_INFO("Path Cell Size: %i",path_cell.size());
		//	ROS_INFO("Path Size: %i",path.size());
			int j = 0;
			for (int i = 0; i<path_cell.size(); i++){
			//	ROS_INFO("I = %i", i);
				path[j] = path_cell[i].coords[1]*map_resolution; //y
				path[j+1] = path_cell[i].coords[0]*map_resolution; //x
		//		ROS_INFO("\n Path Point: X:%f Y:%f",path[j],path[j+1]);
				j += 2;
			}
			std::reverse(path.begin(), path.end());
			//ROS_INFO("Test");
			printMap(path_cell);
			//ROS_INFO("Print path to hard disk");
			return path;

		}

};

//This was just to test if it compiled
/*
int main(){
std::vector<double> path;
PathCreator myPath;
path = myPath.getPath (1, 1, 3, 6);
return 0;
}
*/
