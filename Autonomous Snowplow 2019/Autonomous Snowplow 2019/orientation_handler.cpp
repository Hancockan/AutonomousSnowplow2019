#include "orientation_handler.h"
#include <stdlib.h>
#include <iostream>
#include <functional>
#include <string>
#include <algorithm>

using namespace std;

string buildingString = "";
string mostRecent = "";
string anotherFuckingString = "";
double myfloat = 0.0;

const char *portName = "\\\\.\\COM3";
Serial* arduino = new Serial(portName);

void orientation_handler::write(char c)
{

}

orientation_handler::orientation_handler()
{

}

float orientation_handler::getAngle()
{
	return myfloat;
}

void orientation_handler::write(char* c)
{
	arduino->WriteData(c, 8);
}

void orientation_handler::run()
{
	//set intitial variables
	char incomingData[512] = "";
	int dataLength = 255;
	int readResult = 0;
	this->angle = 0;
	bool firstOpen = false;

	//this is the loop that will constantly get the orientation and run on its own thread
	while (true)
	{
		//read 
		readResult = arduino->ReadData(incomingData, dataLength);
		incomingData[readResult] = 0;
		if (firstOpen)
		{
			if (((string)incomingData).find(']') != -1)
			{
				buildingString += incomingData;
				mostRecent = buildingString;
				buildingString = "";
				firstOpen = false;
				string mysubstring = mostRecent.substr(mostRecent.find('[') + 1, mostRecent.find(']') - 1);
				anotherFuckingString = mysubstring;
				myfloat = stof(anotherFuckingString);
				continue;
			}
		}
		else
		{
			if (((string)incomingData).find('[') != -1)
			{
				firstOpen = true;
			}
		}
		
		buildingString += incomingData;
	}
}
