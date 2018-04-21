#pragma once
#include "Graph.h"
#include <queue>
class Algorithm {
private:
	static std::vector<Edge> breadthFirstSearch(Graph graph, std::queue<Node> &L1, std::queue<Node> &L2, std::vector<Edge> &E0);
	static std::vector<Edge> depthFirstSearch();

public:
	Algorithm();
	static std::vector<Graph> getConnectedComponentWithBFS(Graph graph);
	static std::vector<Graph> getConnectedComponentWithDFS(Graph graph);
};

