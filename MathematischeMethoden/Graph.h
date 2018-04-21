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
	void addNode(Node newNode);
	void addEdge(Edge newEdge);
	Node getNode(int id);
	std::vector<int> getNeighboursID(Node node);
	std::unordered_map<int, Node> getNodes();
};