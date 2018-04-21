#pragma once
#include "Graph.h"
#include <queue>
class Algorithm {
private:
	static void breadthFirstSearch(Graph graph, Graph &connectedComponent, std::queue<Node> &unprocessedNodes);
	static std::vector<Edge> depthFirstSearch();

public:
	Algorithm();
	static std::vector<Graph> getConnectedComponentWithBFS(Graph graph);
	static std::vector<Graph> getConnectedComponentWithDFS(Graph graph);
};

