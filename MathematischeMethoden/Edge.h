#pragma once
#include "Node.h"
class Edge {
private:
	int nodeIDV1;
	int nodeIDV2;
	double weight;
	bool hasWeigt;
	bool beInExistence;
	
public:	
	Edge();
	Edge(int nodeIDV1, int nodeIDV2);
	Edge(int nodeIDV1, int nodeIDV2, double weight);
	int getNodeIDV1();
	int getNodeIDV2();
	double getWeight();
	bool exist();
};
