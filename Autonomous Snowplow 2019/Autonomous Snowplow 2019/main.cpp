#include <ctime>
#include <thread>
#include "lidar_handler.h"
#include "SerialPort.h"
#include "Arduino.h"
#include "local_handler.h"

using namespace std;
char *port_name = "\\\\.\\COM9";

//String for incoming data
char incomingData[256] = "";

int main() {
	lidar_handler lidar;
	local_handler local;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	std::thread local_thread(&local_handler::run, local);
	//local_thread.join();
	//lidar_thread.join();

	Arduino a;
	std::thread angle_thread(&Arduino::run, a);
	
	char* left = { "l" };
	char* forward = { "f" };
	char* right = { "r" };
	while (true)
	{
		cout << a.getAngle() << endl;
		Sleep(100);
		a.write(left);
	}
	angle_thread.join();
	//lidar_thread.join();
	//local_thread.join();
	return 0;
}