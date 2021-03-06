#pragma once
#include "Graph.h"
#include <queue>
class Algorithm {
private:
	static Graph breadthFirstSearch(Graph &graph, int startNode);
	static void depthFirstSearch(Graph &graph, int startNode, std::vector<bool> &visitedNodes);
public:
	Algorithm();
	static std::vector<Graph> getConnectedComponentWithBFS(Graph &graph);
	static int getConnectedComponentWithDFS(Graph &graph);
	static Graph getPrimMinimumSpanningTree(Graph &graph);
	static double getKruskalMinimumSpanningTreeWeight(Graph &graph);
	static Graph getNearestNeighborHamiltonianPath(Graph &graph, int startNodeID);
	static Graph getDoubleTreeHamiltonianPath(Graph &graph, int startNodeID);
	static Graph getOptimalTSP(Graph &graph, bool useBranchBound = true);
	static Graph getShortestPathDijkstra(Graph &graph, int startNodeID);
	static Graph getShortestPathBellmanFord(Graph &graph, int startNodeID, bool &negativeCycleExist);
	static void test(Graph &graph);
	static Graph getMaxFlowEdmondsKarpAlgorithm(Graph &graph, int startNodeID, int targetNodeID);
	static Graph getMinFlowCycleCancelingAlg(Graph &graph);
};