#pragma once
#include "Node.h"
class Edge {
private:
	Node nodeV1;
	Node nodeV2;
public:	
	Edge();
	Edge(Node nodeV1, Node nodeV2);
	Node getNodeV1();
	Node getNodeV2();
};
