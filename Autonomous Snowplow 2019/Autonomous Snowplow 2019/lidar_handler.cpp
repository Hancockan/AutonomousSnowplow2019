#include "lidar_handler.h"

using namespace std;

//global variables
bool bad_scan = false;
vector<string> raw_hex_scan;
vector<tuple<double, double>> raw_decimal_data;
int data_points;

//constructor
lidar_handler::lidar_handler() {

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
		//cout << "distance at half: " << get<1>(raw_decimal_data[data_points/2]) << endl;
		//cout << "angle at half: " << get<0>(raw_decimal_data[data_points/2]) << endl;

		//calculate run time
		double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC * 1000.0;
		//std::cout << "printf: " << duration << '\n';
	}
}

void lidar_handler::analyze_scan() {
	//dont input new data if it was a bad scan
	if (!bad_scan) {
		//clear out old data
		raw_decimal_data.clear();
		//set start angle to fill in data
		double start_angle = -5.0;
		//iterate through all points
		for (int i = 0; i < data_points; i++) {
			//calculate distance
			double distance = convert_to_decimal(raw_hex_scan[26 + i])/1000.0;
			//push distance with its angle into the vector
			raw_decimal_data.push_back(make_tuple(start_angle, distance));
			//increment start angle
			start_angle += 0.5;
		}
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
	addr.sin_addr.s_addr = inet_addr("169.254.144.5");
	addr.sin_family = AF_INET;
	addr.sin_port = htons(2112);

	//declaring the socket object
	SOCKET sock = socket(AF_INET, SOCK_STREAM, NULL);

	//Set a timeout of 50 milliseconds -- some messages take too long to receive
	int timeout = 500;
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
		data_points = convert_to_decimal(results[25]);
		////this will constrict our ability to change angular resolution
		//if (data_points != data_points) {
		//	bad_scan = true;
		//	return;
		//}
		//checking end of scan
		if (results.size() < (data_points + 27) || convert_to_decimal(results[25 + data_points + 1]) != 0) {
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
