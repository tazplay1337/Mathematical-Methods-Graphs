#pragma once
#include "Node.h"
class Edge {
private:
	int nodeIDV1;
	int nodeIDV2;
	double weight;
	double capacity;
	bool hasWeigt;
	bool hasCapacity;
	bool beInExistence;
	
public:	
	Edge();
	Edge(int nodeIDV1, int nodeIDV2);
	Edge(int nodeIDV1, int nodeIDV2, double weight);
	Edge(int nodeIDV1, int nodeIDV2, double cost, double capacity);
	int getNodeIDV1();
	int getNodeIDV2();
	double getWeight();
	void setWeight(double weight);
	void addWeight(double weight);
	bool exist();
};
