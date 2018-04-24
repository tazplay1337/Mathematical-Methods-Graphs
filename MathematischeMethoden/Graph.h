#pragma once
#pragma once
#include "Node.h"
#include "Edge.h"
#include <vector>
#include <unordered_map>

class Graph{
private:
	std::vector<Edge> edges;
	//List<List<
	std::unordered_map<int, Node> nodes; 								
public:
	Graph();
	~Graph();
	void addNode(Node newNode);
	void addEdge(Edge newEdge);
	int edgesSize();
	int nodesSize();
	bool isEmpty();
	bool containNode(int id);
	std::vector<int> getNeighboursID(Node node);
	Node getNode(int id);
	std::unordered_map<int, Node> getNodes();
	std::vector<int> getNodesID();
};