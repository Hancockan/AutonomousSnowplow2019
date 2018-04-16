// This creates a map (named world) that contains a tuple key and Node value that define the world.
//

#include "stdafx.h"
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <list>
#include <tuple>
#include <map>
#include <unordered_map>

using namespace std;

class Node
{
public:
	Node(double x, double y);
	Node();
	double getX();
	double getY();
	void printNodeLocation();
private:
	double x;
	double y;
	tuple<double, double> position;
	std::list<Node> neighborNodes;
	int numLidarHits = 0;
};

Node::Node()
{
	numLidarHits = 0;
	return;
}
Node::Node(double xin, double yin)
{
	position = make_tuple(xin, yin);
	numLidarHits = 0;
}

double Node::getX()
{
	return get<0>(position);
}
double Node::getY()
{
	return get<1>(position);
}
void Node::printNodeLocation()
{
	x = get<0>(position);
	y = get<1>(position);
	printf("(%lf,%lf)", x, y);
}

int main()
{

	double worldSizeX = 10;
	double worldSizeY = 15;
	double resolution = .05;
	map<tuple<double, double>, Node> world;

	for (double x = -worldSizeX / 2; x < worldSizeX / 2; x += resolution)
	{
		for (double y = -worldSizeY / 2; y<worldSizeY / 2; y += resolution)
		{
			tuple<double, double> position = make_tuple(x, y);
			Node newnode = Node(x, y);
			world[position] = newnode;
			world[position].printNodeLocation();
		}
	}

	return 0;
}

