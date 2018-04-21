#pragma once
#include "Graph.h"
class Algorithm {
private:
	std::vector<Edge> breadthFirstSearch(std::vector<Node> &L1, std::vector<Node> &L2, std::vector<Edge> &E0);
	std::vector<Edge> depthFirstSearch();

public:
	Algorithm();
	static std::vector<Graph> getConnectedComponentWithBFS(Graph graph);
	static std::vector<Graph> getConnectedComponentWithDFS(Graph graph);
};

