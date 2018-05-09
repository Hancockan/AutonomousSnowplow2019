//doesn't give warnings for old stuff
#define _WINSOCK_DEPRECATED_NO_WARNINGS

//only include once and also winsock version
#pragma once
#pragma comment(lib, "ws2_32.lib")

//standard includes
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iterator>
#include <tuple>

//includes for websockets -- these are in a certain order
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
#include <ctime>

//winsock version
#define SCK_VERSION2 0x0202

using namespace std;

class lidar_handler {
public:
	lidar_handler();
	void run();
	void perform_scan();
	void analyze_scan();
	int convert_to_decimal(string num);
};

