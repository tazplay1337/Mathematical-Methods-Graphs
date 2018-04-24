#pragma once
#include "Algorithm.h"


Algorithm::Algorithm() {};

int totalNodesSize(std::vector<Graph> &graphs);
int findUnvisitedNodeID(std::vector<Graph> &graphs, Graph &graph);

std::vector<Graph> Algorithm::getConnectedComponentWithBFS(Graph &graph) {
	std::vector<Graph> connectedComponentsOfGraph;
//	Graph connectedComponent;
	std::queue<Node> unprocessedNodes;
	Node startNode;
	int unvisitedNode = 0;

	while(totalNodesSize(connectedComponentsOfGraph) < graph.nodesSize()){
		Graph connectedComponent;
		unvisitedNode = findUnvisitedNodeID(connectedComponentsOfGraph, graph);
		startNode = graph.getNode(unvisitedNode);
		unprocessedNodes.push(startNode);
		connectedComponent.addNode(startNode);
		breadthFirstSearch(graph, connectedComponent, unprocessedNodes);
		connectedComponentsOfGraph.push_back(connectedComponent);
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

int findUnvisitedNodeID(std::vector<Graph> &graphs, Graph &graph) {
	int nodeID = 0;
	bool foundVisitedNodeID = true;
	std::vector<int> getNodesID = graph.getNodesID();

	for (size_t i = 0; foundVisitedNodeID && i < getNodesID.size(); i++){
		nodeID = getNodesID[i];
		foundVisitedNodeID = false;
		for (size_t j = 0; j < graphs.size(); j++) {
			if (graphs[j].containNode(nodeID)) {
				foundVisitedNodeID = true;
				break;
			}
		}
	}
	return nodeID;
}

bool unvisitedNeighboursExist(std::unordered_map<int, Node> &visitedNodes, std::vector<int> &neighboursOfcurrentNode);
int getUnvisitedNeighbourID(std::unordered_map<int, Node> &visitedNodes, std::vector<int> &neighboursOfcurrentNode);

void Algorithm::breadthFirstSearch(Graph graph, Graph &connectedComponent, std::queue<Node> &unprocessedNodes) {
	while (!unprocessedNodes.empty()) {
		Node currentNode = unprocessedNodes.front();
		std::vector<int> neighboursIDsOfcurrentNode = graph.getNeighboursID(currentNode);
		std::unordered_map<int, Node> visitedNodes = connectedComponent.getNodes();
		
		if (!unvisitedNeighboursExist(visitedNodes, neighboursIDsOfcurrentNode)) {
			unprocessedNodes.pop();
		}
		else {
			int unvisitedNodeID = getUnvisitedNeighbourID(visitedNodes, neighboursIDsOfcurrentNode);
			Node unvisitedNode = graph.getNode(unvisitedNodeID);
			unprocessedNodes.push(unvisitedNode);
			connectedComponent.addNode(unvisitedNode);
			Edge newEdge = Edge(currentNode, unvisitedNode);
			connectedComponent.addEdge(newEdge);
		}
	}
}

bool unvisitedNeighboursExist(std::unordered_map<int, Node> &visitedNodes, std::vector<int> &neighboursIDsOfcurrentNode) {
	bool allNeighboursExist = true;
	int neighbourID = 0;

	for (size_t i = 0; allNeighboursExist && i < neighboursIDsOfcurrentNode.size(); i++) {
		neighbourID = neighboursIDsOfcurrentNode[i];

		if (visitedNodes.find(neighbourID) == visitedNodes.end()) {
			allNeighboursExist = false;
		}
	}
	return !allNeighboursExist;
}

int getUnvisitedNeighbourID(std::unordered_map<int, Node> &visitedNodes, std::vector<int> &neighboursOfcurrentNode) {
	bool firstUnvisitedNotFound = true;
	int neighbourID = 0;

	for (size_t i = 0; firstUnvisitedNotFound && i < neighboursOfcurrentNode.size(); i++) {
		neighbourID = neighboursOfcurrentNode[i];

		if (visitedNodes.find(neighbourID) == visitedNodes.end()) {
			firstUnvisitedNotFound = false;
		}
	}
	return neighbourID;
}
/*
std::vector<Edge> Algorithm::depthFirstSearch() {
}
*/

/*
std::vector<Edge> Algorithm::getConnectedComponentWithDFS(Graph graph) {
}*/