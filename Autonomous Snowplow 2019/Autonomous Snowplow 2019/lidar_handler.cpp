/*
When changing to a different lidar you need to change:
- angular resolution increments
- ip address
- port number
- timeout amount in milliseconds (15 Hz would equal 66.6666 ms)
===============================================================
- might be inconsistent with tim551 - can't check end of string

*/
#include "lidar_handler.h"

using namespace std;

//global variables
bool bad_scan = false;
//is there new data for the grid to grab?
bool data_ready = false;
vector<string> raw_hex_scan;
lidar_handler::lidar_data_packet scan_data_pkt;
//vector<tuple<double, double>> raw_decimal_data;
double x_pos, y_pos;

//constructor
lidar_handler::lidar_handler(double x, double y) {
	//when creating the lidar send x and y
	//TODO add orientation
	x_pos = x;
	y_pos = y;
}

void lidar_handler::run() {
	while (1) {
		//for timing the function
		auto start = clock();

		//do the scan and set flag for correctness
		perform_scan();

		//make the data readable
		analyze_scan();

		//print to check
		if (!bad_scan) {
			//cout << endl << "distance at half: " << get<1>((&scan_data_pkt)->raw_data[(&scan_data_pkt)->data_points/2]);
			//cout << "angle at half: " << get<0>(raw_decimal_data[data_points/2]) << endl;
		}

		//calculate run time
		double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC * 1000.0;

		//print run time
		//std::cout << "printf: " << duration << '\n';
	}
}

void lidar_handler::analyze_scan() {
	//dont input new data if it was a bad scan
	if (!bad_scan) {
		//clear out old data
		(&scan_data_pkt)->raw_data.clear();
		//set start angle to fill in data
		double start_angle = -45.0;
		//iterate through all points
		for (int i = 0; i < (&scan_data_pkt)->data_points; i++) {
			//calculate distance
			double distance = convert_to_decimal(raw_hex_scan[26 + i])/1000.0;
			//push distance with its angle into the vector
			(&scan_data_pkt)->raw_data.push_back(make_tuple(start_angle, distance));
			//increment start angle
			start_angle += 1.0;
		}
		//new data is ready for the grid
		data_ready = true;
	}
	else {
		cout << "bad scan" << endl;
		return;
	}
}

void lidar_handler::perform_scan() {
	bad_scan = false;
	//setting up the command
	const char* last_scan;
	string last_scan_string = "";
	last_scan_string += (char)2;
	last_scan_string += "sRN";
	last_scan_string += (char)32;
	last_scan_string += "LMDscandata";
	last_scan_string += (char)3;
	last_scan = last_scan_string.c_str();

	//setting local variables
	long SUCCESSFUL;
	WSADATA WinSockData;
	WORD DLLVersion;
	DLLVersion = MAKEWORD(2, 1);

	//starting up the socket stuff, shutfown if it fails
	if (WSAStartup(DLLVersion, &WinSockData) != 0) {
		bad_scan = true;
		return;
	}

	//setting up variables for receiving data
	string RESPONSE;
	string CONVERTER;
	char MESSAGE[5000];

	//setting up address information
	SOCKADDR_IN addr;
	addr.sin_addr.s_addr = inet_addr("169.254.185.233");
	addr.sin_family = AF_INET;
	addr.sin_port = htons(2111);

	//declaring the socket object
	SOCKET sock = socket(AF_INET, SOCK_STREAM, NULL);

	//Set a timeout of 50 milliseconds -- some messages take too long to receive
	int timeout = 80;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(int));

	//attempt to connect to the lidar and if not successful, shutdown
	if (connect(sock, (SOCKADDR*)&addr, sizeof(addr)) != 0) {
		bad_scan = true;
		return;
	}

	//send the command for last scan
	send(sock, last_scan, (int)strlen(last_scan), 0);

	//receive the data to the message array and set flag
	SUCCESSFUL = recv(sock, MESSAGE, sizeof(MESSAGE), NULL);

	//close the socket
	closesocket(sock);

	//turn the char array into a string, return bad scan if no message
	CONVERTER = MESSAGE;
	
	//prints raw data
	//cout << CONVERTER << endl;

	//report a bad scan if there is no data in the message
	if (CONVERTER.length() == 0) {
		bad_scan = true;
		return;
	}

	//split the data by spaces into an array 
	istringstream iss(CONVERTER);
	vector<string> results((istream_iterator<string>(iss)),istream_iterator<string>());

	//check for correctness (we only want to read good, whole messages)
	if (results.size() > 25) {
		(&scan_data_pkt)->data_points = convert_to_decimal(results[25]);

		//prints number of data poitns
		//cout << "number of data points: " << data_points << endl;

		//prints entities in string
		//cout << "number of entities in string: " << results.size() << endl;

		//checking end of scan
		if (results.size() < ((&scan_data_pkt)->data_points + 27) /*|| convert_to_decimal(results[25 + data_points + 2]) != 0*/) {
			cout << "error string: " << results[25 + (&scan_data_pkt)->data_points + 2]  << endl;
			bad_scan = true;
			return;
		}
	}
	else {
		bad_scan = true;
		return;
	}
	//set raw hex data to the newest scan
	raw_hex_scan = results;
}

//using this for hex conversion
int lidar_handler::convert_to_decimal(string num) {
	int total;
	stringstream ss;
	ss << hex << num;
	ss >> total;
	return total;
}

lidar_handler::lidar_data_packet* lidar_handler::get_data() {
	data_ready = false;
	return (&scan_data_pkt);
}

//referencing raw angle, distance data
bool lidar_handler::data_is_ready() {
	if (data_ready) {
		return true;
	}
	else {
		return false;
	}
}

double lidar_handler::get_x_loc() {
	return x_pos;
}

double lidar_handler::get_y_loc() {
	return y_pos;
}