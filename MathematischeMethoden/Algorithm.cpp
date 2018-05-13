#pragma once
#include "Algorithm.h"
#include "DisjointSets.h"
#include <iostream>
#include <utility>
#include <queue>

Algorithm::Algorithm() {};

int totalNodesSize(std::vector<Graph> &graphs);
int findUnvisitedNodeID(std::vector<bool> &visitedNodes);
void setVisitedNodes(std::vector<bool> &visitedNodes, Graph &graph);

std::vector<Graph> Algorithm::getConnectedComponentWithBFS(Graph &graph) {
	Graph connectedComponent;
	std::vector<Graph> connectedComponentsOfGraph;
	int sizeGraph = graph.sizeNodes();
	int unvisitedNodeID = 0;
	std::vector<bool> visitedNodes(sizeGraph, false);

	while (totalNodesSize(connectedComponentsOfGraph) < sizeGraph) {
		unvisitedNodeID = findUnvisitedNodeID(visitedNodes); //starte vom letzten gefundenen
		connectedComponent = breadthFirstSearch(graph, unvisitedNodeID);
		connectedComponentsOfGraph.push_back(connectedComponent);
		setVisitedNodes(visitedNodes, connectedComponent);
	}
	return connectedComponentsOfGraph;
}

int totalNodesSize(std::vector<Graph> &graphs) {
	int totalSize = 0;

	for (size_t i = 0; i < graphs.size(); i++) {
		totalSize += graphs[i].sizeNodes();
	}
	return totalSize;
}

int findUnvisitedNodeID(std::vector<bool> &visitedNodes) {
	const int ALL_NODES_VISITED = -1;
	int nodeID = ALL_NODES_VISITED;
	bool isNodeVisited = true;

	for (size_t i = 0; isNodeVisited && i < visitedNodes.size(); i++) {
		isNodeVisited = visitedNodes[i];

		if (!isNodeVisited) {
			nodeID = i;
		}
	}
	return nodeID;
}

void setVisitedNodes(std::vector<bool> &visitedNodes, Graph &graph) {
	for (size_t i = 0; i < visitedNodes.size(); i++) {
		if (graph.nodeExist(i)) {
			visitedNodes[i] = true;
		}
	}
}

Graph Algorithm::breadthFirstSearch(Graph &graph, int startNodeID) {
	int currentNodeID = startNodeID;
	int neighbourNode = 0;
	Graph connectedComponent;
	std::queue<int> unprocessedNodes;
	std::vector<int> neighbourIDsOfcurrentNode;
	
	connectedComponent.addNode(currentNodeID);
	unprocessedNodes.push(currentNodeID);
	
	while (!unprocessedNodes.empty()) {
		currentNodeID = unprocessedNodes.front();
		unprocessedNodes.pop();
		neighbourIDsOfcurrentNode = graph.getNeighboursID(currentNodeID);

		for (size_t i = 0; i < neighbourIDsOfcurrentNode.size(); i++) {
			neighbourNode = neighbourIDsOfcurrentNode[i];

			if (!connectedComponent.nodeExist(neighbourNode)) {
				unprocessedNodes.push(neighbourNode);
				connectedComponent.addNode(neighbourNode);
				connectedComponent.addEdge(currentNodeID, neighbourNode);
			}
		}		
	}
	return connectedComponent;
}

int Algorithm::getConnectedComponentWithDFS(Graph &graph) {
	const int cAllNodesVisited = -1;
	int counterConnectedComponent = 0;
	int sizeGraph = graph.sizeNodes();
	int unvisitedNodeID = 0;
	std::vector<bool> visitedNodesOfGraph(sizeGraph, false); //nodesVisited

	while (cAllNodesVisited < unvisitedNodeID) { //besser: for i = letzter unbesuchter Knooten)
		depthFirstSearch(graph, unvisitedNodeID, visitedNodesOfGraph);
		unvisitedNodeID = findUnvisitedNodeID(visitedNodesOfGraph);
		counterConnectedComponent += 1;
	}
	return counterConnectedComponent;
}

void Algorithm::depthFirstSearch(Graph &graph, int nodeID, std::vector<bool> &visitedNodes) {
	int currentNodeID = nodeID;
	int neighbourNode = 0;
	visitedNodes[nodeID] = true;
	std::vector<int> neighbourIDsOfcurrentNode = graph.getNeighboursID(nodeID);

	for (int i = 0; i < neighbourIDsOfcurrentNode.size(); i++) {
		neighbourNode = neighbourIDsOfcurrentNode[i];

		if (!visitedNodes[neighbourNode]) {
			depthFirstSearch(graph, neighbourNode, visitedNodes);
		}
	}
}

class compareEdges {
public:
	double operator() (Edge& p1, Edge& p2) {
		return p1.getWeight() > p2.getWeight();
	}
};

void pushEdgesInPQ(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges, std::vector<Edge> &edges);
Edge poplowestCostEdge(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges);
std::vector<Edge> findCandidateEdges(Node &currentNode, Graph &graph, Graph &mst);

Graph Algorithm::getPrimMinimumSpanningTree(Graph &graph) {
	Graph mst = Graph();
	std::priority_queue<Edge, std::vector<Edge>, compareEdges> candidateEdges;
	std::vector<Edge> candidates;
	int sizeGraph = graph.sizeNodes();
	Edge lowestCostEdge;
	Node newNode = graph.getNode(0);

	mst.addNode(newNode.getID());
	candidates = graph.getNodeEdges(newNode.getID());
	pushEdgesInPQ(candidateEdges, candidates);

	while (mst.sizeNodes() < sizeGraph) {
		lowestCostEdge = poplowestCostEdge(candidateEdges);

		if (!mst.nodeExist(lowestCostEdge.getNodeIDV1()) || !mst.nodeExist(lowestCostEdge.getNodeIDV2())) {
			newNode = !mst.nodeExist(lowestCostEdge.getNodeIDV1()) ? Node(lowestCostEdge.getNodeIDV1()) : Node(lowestCostEdge.getNodeIDV2());
			mst.addNode(newNode);
			mst.addEdge(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2(), lowestCostEdge.getWeight());
			mst.updateNeighbour(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2());
			candidates = findCandidateEdges(newNode, graph, mst);
			pushEdgesInPQ(candidateEdges, candidates);
		}
	}
	return mst;
}

void pushEdgesInPQ(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges, std::vector<Edge> &edges) {
	for (int i = 0; i < edges.size(); i++) {
		candidateEdges.push(edges[i]);
	}
}

Edge poplowestCostEdge(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges) {
	Edge lowestCostEdge = candidateEdges.top();
	candidateEdges.pop();
	return lowestCostEdge;
}

std::vector<Edge> findCandidateEdges(Node &node, Graph &graph, Graph &mst) {
	std::vector<Edge> allEdges = graph.getNodeEdges(node.getID());
	std::vector<Edge> candidate;
	Edge edge;

	for (int i = 0; i < allEdges.size(); i++) {
		edge = allEdges[i];
		if (!mst.nodeExist(edge.getNodeIDV1()) || !mst.nodeExist(edge.getNodeIDV2())) {
			candidate.push_back(edge);
		}
	}
	return candidate;
}

bool your_comparer(Edge left, Edge right) {
	return left.getWeight() < right.getWeight();
}

bool isSelectedEdgeCreatingCycle(int set_u, int set_v);

double Algorithm::getKruskalMinimumSpanningTree(Graph &graph) {
	double mstCosts = 0;
	DisjointSets disjointSets(graph.sizeNodes());
	std::vector<Edge> edges;
	Edge edge;

	graph.copyEdgesInVector(edges);
	std::sort(edges.begin(), edges.end(), your_comparer);
	
	for (int i = 0; i < edges.size(); i++) {
		edge = edges[i];
		int nodeIDV1 = edge.getNodeIDV1();
		int nodeIDV2 = edge.getNodeIDV2();

		int set_node1 = disjointSets.findParentOf(nodeIDV1);
		int set_node2 = disjointSets.findParentOf(nodeIDV2);

		if (isSelectedEdgeCreatingCycle(set_node1, set_node2)) {
			mstCosts += edges[i].getWeight(); // Update MST weight
			disjointSets.mergeTreesLowerToHigher(set_node1, set_node2);
		}
	}
	return mstCosts;
}

bool isSelectedEdgeCreatingCycle(int set_u, int set_v) {
	return set_u != set_v;
}

void  Algorithm::test(Graph &graph) {
	std::vector<Edge> edges;
	graph.copyEdgesInVector(edges);
	std::sort(edges.begin(), edges.end(), your_comparer);

	int i = 0;
}



