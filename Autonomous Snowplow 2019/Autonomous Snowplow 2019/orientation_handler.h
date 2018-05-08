#pragma once
#pragma once
#include <stdlib.h>
#include <stdio.h>
#include "SerialPort.h"

class orientation_handler
{
public:
	orientation_handler();
	void write(char command);
	void run();
	float getAngle();
	void write(char* c);
private:
	float angle;
};