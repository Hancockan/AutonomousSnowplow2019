#include "local_handler.h"

using namespace std;

//global variables
string building_scan = "";
string recent_scan = "";
bool first_open = false;
bool first_close = false;
bool second_open = false;
double x_coord = 0.0;
double y_coord = 0.0;

double local_handler::get_x() {
	return x_coord;
}

double local_handler::get_y() {
	return y_coord;
}

void local_handler::run() {
	Serial* SP = new Serial("\\\\.\\COM6");    // adjust as needed
	char incomingData[256] = "";			   // don't forget to pre-allocate memory
											   //printf("%s\n",incomingData);
	int dataLength = 255;
	int readResult = 0;

	//for testing
	int e = 0;
	bool first_scan = true;

	//run forever once connected to the tag -- this should be in its own thread
	while (SP->IsConnected())
	{
		readResult = SP->ReadData(incomingData, dataLength);
		incomingData[readResult] = 0;

		if (strlen(incomingData) != 0) {
			//cout << "good data: " << incomingData << "e value: " << e << endl;
			e = 0;
		}
		else {
			e++;
		}

		/*
		ignore backup at beginning which is a buffer of a lot of characters
		also this parses using the brackets in the message from the listener
		*/
		if (strlen(incomingData) < 70) {

			if (e > 2000) {
				recent_scan = building_scan;
				//cout << "===========================NEW SCAN===========================" << endl;
				for (int i = 0; i < recent_scan.length(); i++) {
					//cout << recent_scan[i];
				}
				building_scan = "";
				building_scan += incomingData;
	
				bool first_open_bracket = false;
				bool second_open_bracket = false;
				bool x_done = false;
				string x_str;
				string y_str;
				bool y_done = false;
				for (int i = 0; i < recent_scan.length(); i++) {
					if (!first_open_bracket && recent_scan[i] == '[') {
						first_open_bracket = true;
					}
					else if (first_open_bracket && recent_scan[i] == '[') {
						second_open_bracket = true;
					}
					if (first_open_bracket && second_open_bracket && recent_scan[i] == ',') {
						x_done = true;
						continue;
					}
					if (first_open_bracket && second_open_bracket && !x_done) {
						x_str += recent_scan[i];
					}
					if (first_open_bracket && second_open_bracket && x_done && !y_done) {
						y_str += recent_scan[i];
					}
					if (first_open_bracket && second_open_bracket && x_done && recent_scan[i] == ',') {

						y_done = true;

					}

				}
				if (first_scan == false) {
					try {
						for (int i = 0; i < x_str.length(); i++) {
							//cout << x_str[i];
						}
						//cout << endl;
						int x_tot = 4;
						if (x_str.find('-') != -1) {
							x_tot++;
						}
						if (x_str.length() > 0) {
							string x_string = x_str.substr(1, x_tot);
							istringstream i(x_string);
							if (!(i >> x_coord)) {
								cout << "BAD CONVERSION";
							}
						}
						for (int i = 0; i < y_str.length(); i++) {
							//cout << y_str[i];
						}

						int y_tot = 4;
						if (y_str.substr(0, 4).find('-') != -1) {
							y_tot++;
						}
						if (y_str.length() > 0) {
							string y_string = y_str.substr(0, y_tot);
							istringstream i(y_string);
							if (!(i >> y_coord)) {
								cout << "BAD CONVERSION";
							}
						}
					}
					catch (const std::exception &exc)
					{
						// catch anything thrown within try block that derives from std::exception
						std::cerr << exc.what() << endl;
					}
				}
				first_scan = false;
				/*
				cout << "x location: " << endl;
				for (int i = 0; i < x_string.length(); i++) {
					//cout << x_string[i];
				}
				
				cout << endl << "y location: ";

				for (int i = 0; i < y_string.length(); i++) {
					//cout << y_string[i];
				}
				cout << endl;
				*/
			}
			else {
				building_scan += incomingData;
			}



			/*
			if (first_open) {
				if (first_close) {
					if (second_open) {
						if (((string)incomingData).find(']') != -1) {

							//idea: take everything after the closing bracket and add it to the next scan


							building_scan += incomingData;
							recent_scan = building_scan;
							building_scan = "";
							cout << endl << "========================================new location..." << endl;
							for (int i = 0; i < recent_scan.length(); i++) {
								//cout << recent_scan[i];
							}
							//cout << endl;
							first_open = false;
							first_close = false;
							second_open = false;

							//the following is for data parsing
							
							bool first_open_bracket = false;
							bool second_open_bracket = false;
							bool x_done = false;
							string x_str;
							string y_str;
							bool y_done = false;
							for (int i = 0; i < recent_scan.length(); i++) {
								if (!first_open_bracket && recent_scan[i] == '[') {
									first_open_bracket = true;
								}
								else if (first_open_bracket && recent_scan[i] == '[') {
									second_open_bracket = true;
								}
								if (first_open_bracket && second_open_bracket && recent_scan[i] == ',') {
									x_done = true;
									continue;
								}
								if (first_open_bracket && second_open_bracket && !x_done) {
									x_str += recent_scan[i];
								}
								if (first_open_bracket && second_open_bracket && x_done && !y_done) {
									y_str += recent_scan[i];
								}
								if (first_open_bracket && second_open_bracket && x_done && recent_scan[i] == ',') {
								
									y_done = true;
						
								}

							}
							cout << "x location: ";
							for (int i = 0; i < x_str.length(); i++) {
								cout << x_str[i];
							}
							cout << endl << "y location: ";

							for (int i = 0; i < y_str.length(); i++) {
								cout << y_str[i];
							}
							cout << endl;
							
							continue;
						}
					}
					else 
						if (((string)incomingData).find('[') != -1) {
							second_open = true;
						
					}
				}
				else 
					if (((string)incomingData).find(']') != -1) {
						first_close = true;
					
				}
			}
			else 
				if (((string)incomingData).find('[') != -1) {
					first_open = true;
				
			}
			building_scan += incomingData;
		*/
		}

	}
}
