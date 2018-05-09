#include <ctime>
#include <thread>
#include "lidar_handler.h"
#include "local_handler.h"
#include "orientation_handler.h"
#include <math.h>

using namespace std;

int main() {
	
	lidar_handler lidar;
	local_handler local;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	std::thread local_thread(&local_handler::run, local);
	
	//lidar_thread.join();
	//local_thread.join();
	



	/*lidar_handler lidar;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	lidar_thread.join();*/


	orientation_handler a;
	std::thread angle_thread(&orientation_handler::run, a);

	char* left = { "l" };
	char* forward = { "f" };
	char* right = { "r" };
	double xDesired = 3.0;
	double yDesired = 3.0;
	double x;
	double y;
	double heading;
	double headingDesired;
	double smallestDif;
	bool atPoint = false;
	double distToNextWP;
	double distTol = 0.5;
	while (true)
	{
		x = local.getX();
		y = local.getY();
		heading = a.getAngle();
		headingDesired = atan2(yDesired - y, xDesired - x);
		headingDesired = 180.0 / 3.14159*headingDesired;
		smallestDif = headingDesired - heading;
		smallestDif = fmod((smallestDif + 180.0),360.0) - 180;
		atPoint = false;
		distToNextWP = sqrt((x - xDesired)*(x - xDesired) + (y - yDesired)*(y - yDesired));
		if (distToNextWP < distTol)
		{
			cout << "At desired point.  Enter new x, then new y" << endl;
			atPoint = true;
			cin >> xDesired;
			cin >> yDesired;
		}
		else
		{
			if (abs(smallestDif) < 10) //drive forward
			{
				a.write(forward);
			}
			else
			{
				if (smallestDif < 0)
				{
					a.write(right);
				}
				else
				{
					a.write(left);
				}
			}
		}
		//cout << a.getAngle() << endl;
		//Sleep(10);
		//a.write(right);
		

	}
	angle_thread.join();

	return 0;
}