//Hay que poner en el fichero cpp #include "math.h"
class path{
	private:
		double m, k;
		double thresshold = 0.001;
		//y = k-xrob*m + m*x
		
	public:
		double get_m(){return m;}
		double get_k(){return k;}
		void set_m(double m_dest){m = m_dest;}
		void set_k(double k_dest){k = k_dest;}
		int mult = 100;
		
		double** path_points(double x_dest, double y_dest){
	
			//get_robot_position(in the global frame)
	std::cout << "Introduzca xrobot" << std::endl;
			double x_robot = cin >> x;
std::cout << "xrobot introducido: " <<x_robot<< std::endl;
std::cout << "Introduzca yrobot" << std::endl;
			double y_robot = std::cin >> y;
	std::cout << "yrobot introducido: " <<x_robot<< std::endl;
			if (x_dest-thresshold <= x_robot <= x_dest+thresshold){
				length = abs(y_dest-y_robot);
				for (int i = 0, i<= (int)lenght*mult, i++){
					path[i] = {x_robot, i/mult};
				}
		
			}
			else if (y_dest-thresshold <=y_robot <= y_dest+thresshold){
				length = abs(x_dest-x_robot);
				for (int i = 0, i<= (int)lenght*mult, i++){
					path[i] = {i/mult, y_robot};
				}
		
			}
		
			else{
				length = sqrt(pow(x_dest-x_rob, 2)+pow(y_dest-y_rob, 2));
		/*		set_m((ydest-y_robot)/(xdest-x_robot));
				set_k(y_robot);
				double phi = atan2((y_dest-y_rob)/(x_dest-x_rob));
*/
				for (int i = 0, i<= (int)lenght*mult, i++){
					path[i] = {(i/mult*cos(phi)), (i/mult*sin(phi))};
				}
			}
	
			return path;
		} 
		
	
};

