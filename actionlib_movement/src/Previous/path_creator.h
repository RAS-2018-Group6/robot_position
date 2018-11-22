//Hay que poner en el fichero cpp #include "math.h"
#ifndef path_creator.h
#define path_creator.h
class path{
	private:
		double m, k;
		const double thresshold = 0.001;
		//y = k-xrobot*m + m*x
		int number;
		
	public:
		double get_m(){return m;}
		double get_k(){return k;}
		void set_m(double m_dest){m = m_dest;}
		void set_k(double k_dest){k = k_dest;}
		const int mult = 100;

		int getNumber(){return number;}
		
		double** path_pts(double x_dest, double y_dest, double x_robot, double y_robot){
	
			//get_robot_position(in the global frame)
			double** path_points = 0; 
			double length = 0;

			if (x_dest-thresshold <= x_robot <= x_dest+thresshold){
				length = abs(y_dest-y_robot);
				for (int i = 0; i<= ((int)(length*mult)); i++){
					path_points[i][0] = x_robot;
					path_points[i][1] = i/mult;
				}
		
			}
			else if (y_dest-thresshold <=y_robot <= y_dest+thresshold){
				length = abs(x_dest-x_robot);
				for (int i = 0; i<= ((int)(length*mult)); i++){
					path_points[i][0] = i/mult;
					path_points[i][1] = y_robot;
				}
		
			}
		
			else{
				length = sqrt(pow(x_dest-x_robot, 2)+pow(y_dest-y_robot, 2));
		/*		set_m((ydest-y_robot)/(xdest-x_robot));
				set_k(y_robot);*/
				double phi = atan2((y_dest-y_robot),(x_dest-x_robot));

				for (int i = 0; i<= (int)(length*mult); i++){
					path_points[i][0] = (i/mult*cos(phi));
					path_points[i][1] = (i/mult*sin(phi));
				}
			}
			number = (int)(length*mult);
			return path_points;
		} 
		
	
};
#endif
