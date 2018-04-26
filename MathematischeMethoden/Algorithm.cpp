#pragma once
#include "Algorithm.h"
#include <iostream>

Algorithm::Algorithm() {};

int totalNodesSize(std::vector<Graph> &graphs);
int findUnvisitedNodeID(std::vector<bool> &visitedNodes);
void setVisitedNodes(std::vector<bool> &visitedNodes, Graph &graph);

std::vector<Graph> Algorithm::getConnectedComponentWithBFS(Graph &graph) {
	Graph connectedComponent;
	std::vector<Graph> connectedComponentsOfGraph;
	int sizeGraph = graph.nodesSize();
	int unvisitedNodeID = 0;
	std::vector<bool> visitedNodes(sizeGraph, false);

	while (totalNodesSize(connectedComponentsOfGraph) < sizeGraph) {
		unvisitedNodeID = findUnvisitedNodeID(visitedNodes);
		connectedComponent = breadthFirstSearch(graph, unvisitedNodeID);
		connectedComponentsOfGraph.push_back(connectedComponent);
		setVisitedNodes(visitedNodes, connectedComponent);
	}
	return connectedComponentsOfGraph;
}

int totalNodesSize(std::vector<Graph> &graphs) {
	int totalSize = 0;

	for (size_t i = 0; i < graphs.size(); i++) {
		totalSize += graphs[i].nodesSize();
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
	const int ALL_NODES_VISITED = -1;
	const int FOUND_COMPONENT = 1;
	int counterConnectedComponent = 0;
	int sizeGraph = graph.nodesSize();
	int unvisitedNodeID = 0;
	std::vector<bool> visitedNodesOfGraph(sizeGraph, false);

	while (ALL_NODES_VISITED < unvisitedNodeID) {
		depthFirstSearch(graph, unvisitedNodeID, visitedNodesOfGraph);
		unvisitedNodeID = findUnvisitedNodeID(visitedNodesOfGraph);
		counterConnectedComponent += FOUND_COMPONENT;
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