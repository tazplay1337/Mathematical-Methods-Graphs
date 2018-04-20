#pragma once
#pragma once
#include <unordered_map>

class Graph{
private:
	using Node = int;
	using Edge = int;
	using EdgeList = std::vector<Edge>;
	using AdjencyList = std::unordered_map<int, EdgeList>;

	// Node hashtable = hashtable von nodes
	// Edge Liste = array von edge

	AdjencyList adjList;
public:
	Graph();
	Graph(AdjencyList initialAdjList);
	~Graph();
	void insertNode(int newNode);
	void insertEdge(int v1, int v2);
};