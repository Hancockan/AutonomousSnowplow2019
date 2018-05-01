#include <ctime>
#include <thread>
#include "lidar_handler.h"
#include "SerialPort.h"
#include "Arduino.h"

using namespace std;
char *port_name = "\\\\.\\COM9";

//String for incoming data
char incomingData[256] = "";

int main() {
	/*lidar_handler lidar;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	lidar_thread.join();*/


	Arduino a;
	std::thread angle_thread(&Arduino::run, a);
	
	char* left = { "l" };
	char* forward = { "f" };
	char* right = { "r" };
	while (true)
	{
		cout << a.getAngle() << endl;
		Sleep(10);
		a.write(right);
	}
	angle_thread.join();
	return 0;
}