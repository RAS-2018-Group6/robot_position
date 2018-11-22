#include <stdio.h>
#include<ros/ros.h>
#include <cmath>
#include <vector>
#include "class_cell.hpp"


#define EMPTY -1
#define VALUE 5
#define HIGH_COST 10
#define FREE 0

class MapNode{
	public:
		int nRows = 20;	
		int nColumns = 20;	
		int map_resolution = 20;	
		
		int mToCell(double x){return round(x/map_resolution);}
		std::vector<std::vector<int> > coords;
	
};

class pathCreator{ 

	private:
		MapNode map;
	
	public:

		cell new_cell(int coords[2], double f, double g){

			cell newcell;
			newcell.coords[0]=coords[0];
			newcell.coords[1]=coords[1];
			newcell.f=f;
			newcell.g=g;
			newcell.parent_coords[0]=EMPTY;
			newcell.parent_coords[1]=EMPTY;

			return newcell;

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
		}


		std::vector<cell> astar(int x_robot, int y_robot, int x_goal, int y_goal){

			int lengthx = map.nColumns; 
			int lengthy = map.nRows; //Number of cells in each direction that the map has
			int goal_coords[2]={x_goal,y_goal},coords[2],before[2],cost=0;
			double g, f, h;

			std::vector<cell> not_valid;
	
			g = 0;
			h = heuristic(coords, goal_coords);
			f = g+h;
	
			//First, we check that the goal coordinates are within the limits of the map:
	
			if(x_goal < 0 || y_goal < 0 || x_goal >= lengthx || y_goal >= lengthy){

				ROS_ERROR("The coordinates are not within the limits");

				return not_valid;
			}
	
	
	
	
			cell now,trial;

			coords[0]=x_robot;
			coords[1]=y_robot;
			before[0]=EMPTY;
			before[1]=EMPTY;
	
			//Create the lists of points to accumulate checkpoints
	
			set_of_cells close; //In the beginning it must be empty
			set_of_cells open(now); //The first element must be the first cell to check (the one in which the robot is)
			set_of_cells before_list; //The list of cells that can still be checked

			now = new_cell (coords, f, g);

			not_valid.push_back(now); //Put in the not valid vector the values of the current position
	
			while(!open.empty()) {//While there are still cells to check in the map
				now=open.best_cell(); //save in now the cell with the lowest cost function (f) of the open list
		
				if(now.coords[0]==goal_coords[0] && now.coords[1]==goal_coords[1]){ //If I already have the solution for the path:

					return build_path(now, &close); //Return the path (calling the function build_path

				}
		
				//If I haven't found the path yet, try the cells next to the cell now:
				close.add(now); //Add the cell I'm in to the list
		
				if(now.coords[0]-1>=0){ //If I'm not in the limit 0 of x:
					coords[0]=now.coords[0]-1; //try new coordinates
					coords[1]=now.coords[1]; 

					if(map.coords[coords[0]][coords[1]] == FREE){ //If that cell is free of obstacles
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
		
				if(now.coords[0]+1<=lengthx){ //If I'm not in the limit legth of x:
					coords[0]=now.coords[0]+1; //try new coordinates
					coords[1]=now.coords[1]; 

					if(map.coords[coords[0]][coords[1]] == FREE){ //If that cell is free of obstacles
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
		
				if(now.coords[1]-1>=0){ //If I'm not in the limit 0 of y:
					coords[0]=now.coords[0]; //try new coordinates
					coords[1]=now.coords[1]-1; 

					if(map.coords[coords[0]][coords[1]] == FREE){ //If that cell is free of obstacles
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
		
				if(now.coords[0]+1<=lengthy){ //If I'm not in the limit length of y:
					coords[0]=now.coords[0]; //try new coordinates
					coords[1]=now.coords[1]+1; 

					if(map.coords[coords[0]][coords[1]] == FREE){ //If that cell is free of obstacles
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
			
			}
			return not_valid; //If we are here, it means that there is no path to that point (the while loop has finished and no path has been found)
		}
		
		std::vector<double> getPath (double x_robot, double y_robot, double x_dest, double y_dest){
			//Convert coordinates into cells and send to astar:
			std::vector<cell> path_cell;
			std::vector<double> path;
			
			path_cell = astar(map.mToCell(x_robot),map.mToCell(y_robot),map.mToCell(x_dest),map.mToCell(y_dest));
			
			for (int i = 0; i<path_cell.size(); i+=2){
				path[i] = path_cell[i].coords[0]*map.map_resolution;
				path[i+1] = path_cell[i+1].coords[1]*map.map_resolution;
			}

			return path;

		}

};

//This was just to test if it compiled
/*
int main(){
std::vector<double> path;
pathCreator myPath;
path = myPath.getPath (1, 1, 3, 6);
return 0;
}
*/
