#include <ctime>
#include <thread>
#include "lidar_handler.h"
#include "local_handler.h"
#include "orientation_handler.h"
#include "Grid.h"

using namespace std;

int main() {
	/*lidar we are using with starting coordinates*/
	lidar_handler lidar(2.5, 0.0);
	
	/*create the grid with a reference to the lidar that it will display hits for*/
	Grid Grid_ob(5.0, 5.0, &lidar);

	//local_handler local;

	/*start the threads for the lidar and the grid*/
	std::thread lidar_thread(&lidar_handler::run, lidar);
	std::thread grid_thread(&Grid::run, Grid_ob);

	//std::thread local_thread(&local_handler::run, local);
	
	lidar_thread.join();
	//local_thread.join();

	//orientation_handler a;
	//std::thread angle_thread(&orientation_handler::run, a);

	//angle_thread.join();

	return 0;
}