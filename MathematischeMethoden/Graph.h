#pragma once
#pragma once
#include "Node.h"
#include "Edge.h"
#include <vector>
#include <unordered_map>

class Graph{
private:
	std::vector<Edge> allEdgesOfG;
	std::unordered_map<int, Node> allNodesOfG;
public:
	Graph();
	~Graph();
	void insertNode(Node newNode);
	void insertEdge(Edge newEdge);
};