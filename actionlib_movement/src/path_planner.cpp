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
#define VALUE 5
//#define HIGH_COST 10
#define FREE 30 //Asume there is nothing if the value is less than FREE
#define UNKNOWN -7
#define WALL 70
#define THICK 50
#define ROBSIZE 7 //Number of cells that we thicken the walls

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
		int size1 = 243;
		std::vector<std::vector<signed char> > map;



	public:

		//constructor
		PathCreator() : map(243){;}

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
			for(int i=0;i<nColumn;,i++){
				for(int j=0;j<nRows;j++){
					if(map[i][j] <= FREE){
						output[i][j] = '.';
					}
					else if(map[i][j] >= WALL){
						output[i][j] = '#';
					}
				}
			}
			ofstream myfile;
			myfile.open("map_visualization.txt");
			for(int j=0;j<nRows;j++){
				for(int i=0;i<nColumns;i++){
					myfile << output[i][j];
				}
				myfile << "\n";
			}
			myfile.close();
			path_size = path.size();
			for(int k=0;k<path_size;k+=2){
					int i = path[k].coords[0];
					int j = path[k+1].coords[1];
					output[i][j] = 'x';
			}
			ofstream myfile_2;
			myfile_2.open("path_visualization.txt");
			for(int j=0;j<nRows;j++){
				for(int i=0;i<nColumns;i++){
					myfile_2 << output[i][j];
				}
				myfile_2 << "\n";
			}
			myfile_2.close();
		}

//This function is already in Kristian's code
		int mToCell(float x)
    {
				//ROS_INFO("Original Value: %f",x);

				//ROS_INFO("Map Resolution: %f", map_resolution);
        // converts x from meters to grid cell coordinate
				//ROS_INFO("Value:%i",(int) round(x/map_resolution));
        return (int) round(x/map_resolution);
    }

		void mapMatrix(){ //Converts the data into a matrix
			for (int i =0; i<nColumns; i++){
				//ROS_INFO("%i",i);
				map[i].resize(nRows, UNKNOWN); //Give the vectors of the matrix a length
			}
			//ROS_INFO("TEst");
			for(int i = 0; i<nColumns; i++){
				for (int j = 0; j < nRows; j++){
					map[i][j]=data[i+j*nColumns];
				}
			}
		}

//Maybe this function should be in another place, or do partial smoothing
//For now I'll do a simple function for it here
//This function is not working as it should: it gives the value thicken to all the empty cells
		void smoothMap(){
			for (int i = 0; i < nColumns; i++){
				for (int j = 0; j<nRows; j++){
					//ROS_INFO("nEXT");
					if (map[i][j]>WALL){
						//Make the wall thicker (put a value <WALL and >FREE)
						for (int m = -ROBSIZE; m <= ROBSIZE; m++){
							//ROS_INFO("iNSIDE THE NEXT FOR, M = %i, i = %i, j = %i", m, i, j);
							if (i>=(-m) && (i+m)<(nColumns) && map[i+m][j]<WALL){
								map[i+m][j]=THICK;
							}
						}

						for (int m = -ROBSIZE; m <= ROBSIZE; m++){
							if (j>=(-m) && (j+m)<(nRows) && map[i][j+m]<WALL){
								map[i][j+m]=THICK;
							}
						}
					}
					//ROS_INFO("Out of if, i = %i,  j= %i ", i, j);
				}
			}
			ROS_INFO("Finished smoothing");
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
									ROS_INFO("End of build path");
				return path_vector;
		}


		std::vector<cell> astar(int x_robot, int y_robot, int x_goal, int y_goal){

			int lengthx = nColumns;
			int lengthy = nRows; //Number of cells in each direction that the map has
			int goal_coords[2]={x_goal,y_goal},coords[2],before[2],cost=0;
			double g, f, h;


			std::vector<cell> not_valid;
			mapMatrix();

			smoothMap();
			ROS_INFO("Start A*");

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

				ROS_INFO("The coordinates are not within the limits");

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
					ROS_INFO("Path is built. goal is %i, %i", goal_coords[0],goal_coords[1]);
					return build_path(now, &close); //Return the path (calling the function build_path)


				}

				//If I haven't found the path yet, try the cells next to the cell now:
				close.add(now); //Add the cell I'm in to the list
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
				if(now.coords[0]+1<lengthy){ //If I'm not in the limit length of y:
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
			//	ROS_INFO("aFTER IF NUMBER 4");
			}
			ROS_INFO("nO PATH");

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

			int size_path = path_cell.size()*2;
			std::vector<float> path (size_path);

			//ROS_INFO("Path Cell Size: %i",path_cell.size());
		//	ROS_INFO("Path Size: %i",path.size());

			for (int i = 0; i<path_cell.size(); i+=2){
			//	ROS_INFO("I = %i", i);
				path[i] = path_cell[i].coords[0]*map_resolution;
				path[i+1] = path_cell[i+1].coords[1]*map_resolution;
				ROS_INFO("\n Path Point: X:%f Y:%f",path[i],path[i+1]);
			}
			std::reverse(path.begin(), path.end());
			//ROS_INFO("Test");
			printMap(path_cell);
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
