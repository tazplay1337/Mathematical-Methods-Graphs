#pragma once
#pragma once
#include "Node.h"
#include "Edge.h"
#include <string>
#include <unordered_map>
#include <list>
#include <utility>

class Graph{
private:
	std::unordered_map<std::string, Edge> edges;
	std::unordered_map<int, Node> nodes;
//	std::vector<std::pair<double, Edge>> edges;
//	std::vector< std::pair<int, iPair> > edges;
public:
	Graph();
	~Graph();
	bool isEmpty();
	bool nodeExist(int id);
	int sizeEdges();
	int sizeNodes();
	Node getNode(int id);
	Edge getEdge(int nodeID1, int nodeID2);
	std::unordered_map<int, Node> getNodes();
	std::vector<int> getNodesID();
	std::vector<int> getNeighboursID(int nodeID);
	std::vector<Edge> getNodeEdges(int nodeID);
	void addNode(Node newNode);
	void addEdge(int Node1, int Node2);
	void addEdge(int Node1, int Node2, double weight);
	double totalCost();
	void updateNeighbour(int firstNode, int secondNode);


	void copyEdgesInVector(std::vector<Edge> &edges);
};