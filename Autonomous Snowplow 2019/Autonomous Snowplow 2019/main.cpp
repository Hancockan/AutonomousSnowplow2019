#include <ctime>
#include <thread>
#include "lidar_handler.h"

using namespace std;

int main() {

	lidar_handler lidar;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	lidar_thread.join();



	return 0;
}