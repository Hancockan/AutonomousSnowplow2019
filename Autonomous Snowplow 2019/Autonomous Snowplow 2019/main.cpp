#include <ctime>
#include <thread>
#include "lidar_handler.h"
#include "local_handler.h"

using namespace std;

int main() {

	lidar_handler lidar;
	local_handler local;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	std::thread local_thread(&local_handler::run, local);
	
	lidar_thread.join();
	//local_thread.join();

	return 0;
}