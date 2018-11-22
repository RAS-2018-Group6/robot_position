#ifndef class_cell
#define class_cell

#include<vector>

typedef struct cell{
	
	int coords[2];
	int parent_coords[2];
	int f;
	int g;
	
}cell;

class set_of_cells{
	private:
		std::vector<cell> cell_array; //vector whose elements are of the type "cell" (struct above)
		
	public:
		//Constructor of the class, with a cell in it from the beginning
		set_of_cells(cell newcell){add(newcell);}
		set_of_cells(){;} //For creating an empty set
		
		//Function that adds a new element to the cell array
		void add (cell newcell){
			cell_array.push_back(newcell);
		}
		
		//Function that removes an element of the cell array, knowing the coordinates of the cell
		void remove (int coord_remove[2]){
			for (std::vector<cell>::iterator i = cell_array.begin(); i < cell_array.end(); i++){
				//Look at all the elements from the cells vector until you find the coordinates of the one you want to erase
				if (i -> coords [0] == coord_remove[0] && i -> coords [1] == coord_remove[1]){
					cell_array.erase(i); //Erase that element from the vector
				}
			}
		}
		
		//Function that gives the cell that has been extracted
		cell get (int coord_get[2]){
			for (int i = 0; i < cell_array.size(); i++){
				//Look at all the elements from the cells vector until you find the coordinates of the one you want to erase
				if (cell_array[i].coords [0] == coord_get[0] && cell_array[i].coords [1] == coord_get[1]){
					return cell_array[i];
				}
			}
		}
		
		//Function that gives the best next cell
		cell best_cell(){
			cell best_cell = cell_array [0]; //Initialize with the value of the first element
			for (int i = 0; i<cell_array.size(); i++){
				//If you find another element of the cell array with a lower f, you have new best.
				if (cell_array[i].f < best_cell.f) best_cell = cell_array[i];
			}	
			//This new best should be removed from the array, as it is already being checked
			remove (best_cell.coords);
			
			return best_cell;
		}

		//Function that returns a 1 if the list is empty
		bool empty(){
			if (cell_array.size() == 0) return true;
			return false;
		}
		
		cell pop (int coord_get[2]){
			cell popped;
			for (int i = 0; i < cell_array.size(); i++){
				//Look at all the elements from the cells vector until you find the coordinates of the one you want to erase
				if (cell_array[i].coords [0] == coord_get[0] && cell_array[i].coords [1] == coord_get[1]){
					
					popped =  cell_array[i];
				}
			}
			
			remove(popped.coords);
			return popped;
		}
		
		bool inside(int coords[2]){
			for(int i=0; i<cell_array.size(); i++){
				if(cell_array[i].coords[0] == coords[0] && cell_array[i].coords[1] == coords[1])
					return true;

			}
			return false;
		}
		
		//I haven't done a function to pop one node (would be get+remove)
		//Also, I don't have check if in set: returns true when the coordinates of a cell are in the array

		
		
};

#endif
