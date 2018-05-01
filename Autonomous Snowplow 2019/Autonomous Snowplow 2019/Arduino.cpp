#include "Arduino.h"
#include <stdlib.h>
#include <iostream>
#include <functional>
#include <string>
#include <algorithm>
using namespace std;

string buildingString = "";
string mostRecent = "";
string anotherFuckingString = "";
float myfloat = 0.0;

const char *portName = "\\\\.\\COM9";
Serial* arduino = new Serial(portName);

void Arduino::write(char c)
{

}

Arduino::Arduino()
{

}

float Arduino::getAngle()
{
	return myfloat;
}

void Arduino::write(char* c)
{
	arduino->WriteData(c, 8);
}

void Arduino::run()
{
	
	char incomingData[512] = "";
	int dataLength = 255;
	int readResult = 0;
	this->angle = 0;
	bool firstOpen = false;
	while (true)
	{
		readResult = arduino->ReadData(incomingData, dataLength);
		// printf("Bytes read: (0 means no data available) %i\n",readResult);
		incomingData[readResult] = 0;
		if (firstOpen)
		{
			if (((string)incomingData).find(']') != -1)
			{
				buildingString += incomingData;
				mostRecent = buildingString;
				buildingString = "";
				firstOpen = false;
				/*for (int i = 0; i < mostRecent.length(); i++)
				{
					cout << mostRecent[i];
				}
				cout << endl;*/
				string mysubstring = mostRecent.substr(mostRecent.find('[')+1, mostRecent.find(']')-1);
				if (mysubstring.find(']' != -1))
				{
					mysubstring = mysubstring.substr(0, mysubstring.find(']') - 1);
				}
				anotherFuckingString = mysubstring;
				myfloat = stof(anotherFuckingString);
				//cout << mysubstring << endl;
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
		
		//cout << myfloat << endl;
		//string mystr = incomingData;
		//printf("%s\n", mystr);
		//if ((incomingData[0] == '[') && (incomingData[mystr.length()-1] == ']'))
		//{
		//	float myAngle = stof(mystr.substr(1, mystr.length() - 2));
		//	//printf("%f\n", myAngle);
		//}
		//else
		//{
		//	//printf("%c", mystr[mystr.length()-1]);
		//}
		//Sleep(10);
	}
}

