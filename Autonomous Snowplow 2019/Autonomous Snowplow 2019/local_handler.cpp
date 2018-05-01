#include "local_handler.h"

using namespace std;

//global variables
string building_scan = "";
string recent_scan = "";
bool first_open = false;
bool first_close = false;
bool second_open = false;

void local_handler::run() {
	Serial* SP = new Serial("\\\\.\\COM3");    // adjust as needed
	char incomingData[256] = "";			   // don't forget to pre-allocate memory
											   //printf("%s\n",incomingData);
	int dataLength = 255;
	int readResult = 0;

	//run forever once connected to the tag -- this should be in its own thread
	while (SP->IsConnected())
	{
		readResult = SP->ReadData(incomingData, dataLength);
		incomingData[readResult] = 0;

		/*
		ignore backup at beginning which is a buffer of a lot of characters
		also this parses using the brackets in the message from the listener
		*/
		if (strlen(incomingData) < 70) {
			if (first_open) {
				if (first_close) {
					if (second_open) {
						if (((string)incomingData).find(']') != -1) {
							building_scan += incomingData;
							recent_scan = building_scan;
							building_scan = "";
							//cout << endl << "new location..." << endl;
							for (int i = 0; i < recent_scan.length(); i++) {
								cout << recent_scan[i];
							}
							cout << endl;
							first_open = false;
							first_close = false;
							second_open = false;
							continue;
						}
					}
					else {
						if (((string)incomingData).find('[') != -1) {
							second_open = true;
						}
					}
				}
				else {
					if (((string)incomingData).find(']') != -1) {
						first_close = true;
					}
				}
			}
			else {
				if (((string)incomingData).find('[') != -1) {
					first_open = true;
				}
			}
			building_scan += incomingData;
		}

	}
}
