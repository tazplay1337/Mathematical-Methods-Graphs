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
	bool edgesDirected;
	std::unordered_map<std::string, Edge> edges;
	std::unordered_map<int, Node> nodes;
	void updateNeighbour(int firstNode, int secondNode);
	double valTotalFlow;

public:
	Graph();
	Graph(bool edgesDirected);
	Graph(bool edgesDirected, size_t size, std::unordered_map<std::string, Edge> edges, double valFlow);
	~Graph();
	bool isEmpty();
	bool isDirected();
	bool nodeExist(int id);
	int sizeEdges();
	int sizeNodes();
	Node getNode(int id);
	Edge getEdge(int nodeID1, int nodeID2);
	double getEdgeCost(int nodeID1, int nodeID2);
	std::unordered_map<int, Node> getNodes();
	std::unordered_map<std::string, Edge> getEdges();
	std::vector<int> getNodesID();
	std::vector<int> getNeighboursID(int nodeID);
	std::vector<Edge> getNodeEdges(int nodeID);
	void addNode(Node newNode);
	void addEdge(int Node1, int Node2);
	void addEdge(int Node1, int Node2, double weight);
	void addEdge(int Node1, int Node2, double weight, double capacity);
	double totalCost();
	double getValTotalFlow();
	bool hasNegativeCostEdge();
	void printEdges();


	void copyEdgesInVector(std::vector<Edge> &edges);
};