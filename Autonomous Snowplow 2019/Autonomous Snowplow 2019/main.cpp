#include <ctime>
#include <thread>
#include "lidar_handler.h"
#include "local_handler.h"
#include "orientation_handler.h"
#include <math.h>

using namespace std;

int main() {

	//lidar_handler lidar;
	local_handler local;
	//std::thread lidar_thread(&lidar_handler::run, lidar);
	std::thread local_thread(&local_handler::run, local);

	//lidar_thread.join();
	//local_thread.join();




	/*lidar_handler lidar;
	std::thread lidar_thread(&lidar_handler::run, lidar);
	lidar_thread.join();*/


	orientation_handler a;
	std::thread angle_thread(&orientation_handler::run, a);
	Sleep(3000);
	try {
		char* left = { "l" };
		char* forward = { "f" };
		char* right = { "r" };
		char* stop = { "s" };
		double xDesired = 2.0;
		double yDesired = 1.0;
		double x;
		double y;
		double heading;
		double headingDesired;
		double smallestDif;
		bool atPoint = false;
		double distToNextWP;
		double distTol = 0.5;
		int counter = 0;
		while (true)
		{
			Sleep(10);
			
			x = local.get_x();
			y = local.get_y();
			heading = a.getAngle();
			if (counter % 20 == 0)
			{
				headingDesired = atan2(yDesired - y, xDesired - x);
				headingDesired = 180.0*headingDesired / 3.14159;
			}
			counter++;
			if (headingDesired < 0) {
				headingDesired = 360 + headingDesired;
			}
			smallestDif = headingDesired - heading;
			//smallestDif = fmod((smallestDif + 180.0), 360.0) - 180;
			cout << x << " " << y << " " << heading << " " << headingDesired << " " << smallestDif << endl;
			atPoint = false;
			distToNextWP = sqrt((x - xDesired)*(x - xDesired) + (y - yDesired)*(y - yDesired));
			if (distToNextWP < distTol)
			{
				cout << "At desired point.  Enter new x, then new y" << endl;
				atPoint = true;
				a.write(stop);
				cin >> xDesired;
				cin >> yDesired;

			}
			else
			{
				if (abs(smallestDif) < 15) //drive forward
				{
					a.write(forward);
					cout << "Forward" << endl;
				}
				else
				{
					if (smallestDif < 0)
					{
						a.write(right);
						cout << "Right" << endl;
					}
					else
					{
						a.write(left);
						cout << "Left" << endl;
					}
				}
			}
			//cout << a.getAngle() << endl;
			//Sleep(10);
			//a.write(right);


		}
		angle_thread.join();
	}
	catch (const std::exception &exc)
	{
		// catch anything thrown within try block that derives from std::exception
		std::cerr << exc.what() << endl;
	}

	return 0;
}