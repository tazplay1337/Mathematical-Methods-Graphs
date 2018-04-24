#pragma once
#pragma once
#include "Node.h"
#include "Edge.h"
#include <string>
#include <unordered_map>

class Graph{
private:
//	std::vector<Edge> edges;
	std::unordered_map<std::string, Edge> edges;
	std::unordered_map<int, Node> nodes; 								
public:
	Graph();
	~Graph();
	void addNode(Node newNode);
	void addEdge(int Node1, int Node2);
	int edgesSize();
	int nodesSize();
	bool isEmpty();
	bool containNode(int id);
	std::vector<int> getNeighboursID(Node node);
	Node getNode(int id);
	std::unordered_map<int, Node> getNodes();
	std::vector<int> getNodesID();
};