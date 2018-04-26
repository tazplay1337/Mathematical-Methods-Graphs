#pragma once
#pragma once
#include "Node.h"
#include "Edge.h"
#include <string>
#include <unordered_map>

class Graph{
private:
	std::unordered_map<std::string, Edge> edges;
	std::unordered_map<int, Node> nodes; 								
public:
	Graph();
	~Graph();
	bool isEmpty();
	bool nodeExist(int id);
	int edgesSize();
	int nodesSize();
	Node getNode(int id);
	std::unordered_map<int, Node> getNodes();
	std::vector<int> getNodesID();
	std::vector<int> getNeighboursID(int nodeID);
	void addNode(Node newNode);
	void addEdge(int Node1, int Node2);
	void updateNeighbour(int firstNode, int secondNode);
};