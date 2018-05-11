#pragma once
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"

using namespace std;

class local_handler {
public:
	void run();
	double get_x();
	double get_y();
};