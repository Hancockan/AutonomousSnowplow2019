#pragma once
#include <iostream>
#include <vector>
#include "lidar_handler.h"

using namespace std;

class Grid {
public:
	Grid(double width, double height, lidar_handler *lidar);
	void run();
	void print_hit_map();
	void update_hit_map();
	void update_obj_map();
};