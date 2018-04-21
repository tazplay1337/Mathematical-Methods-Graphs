#pragma once
#pragma once
#include "Node.h"
#include "Edge.h"
#include <vector>
#include <unordered_map>

class Graph{
private:
	std::vector<Edge> edges;
	std::unordered_map<int, Node> nodes;
public:
	Graph();
	~Graph();
	bool isEmpty();
	void insertNode(Node newNode);
	void insertEdge(Edge newEdge);
	Node getNode(int id);
};