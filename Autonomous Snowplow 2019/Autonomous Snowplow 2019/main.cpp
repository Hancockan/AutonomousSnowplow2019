#include <ctime>
#include <thread>
#include "lidar_handler.h"
#include "local_handler.h"
#include "orientation_handler.h"

using namespace std;

int main() {
	/*
	lidar_handler lidar;
	local_handler local;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	//std::thread local_thread(&local_handler::run, local);
	
	lidar_thread.join();
	//local_thread.join();
	*/



	/*lidar_handler lidar;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	lidar_thread.join();*/


	orientation_handler a;
	std::thread angle_thread(&orientation_handler::run, a);

	char* left = { "l" };
	char* forward = { "f" };
	char* right = { "r" };
	while (true)
	{
		cout << a.getAngle() << endl;
		//Sleep(10);
		//a.write(right);
	}
	angle_thread.join();

	return 0;
}