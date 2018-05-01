#pragma once
#include <stdlib.h>
#include <stdio.h>
#include "SerialPort.h"

class Arduino
{
public:
	Arduino();
	void write(char command);
	void run();
	float getAngle();
	void write(char* c);
private:
	float angle;
};