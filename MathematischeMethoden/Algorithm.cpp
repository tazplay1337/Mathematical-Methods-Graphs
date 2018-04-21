#pragma once
#include "Algorithm.h"


Algorithm::Algorithm() {};

std::vector<Graph> Algorithm::getConnectedComponentWithBFS(Graph graph) {
	std::vector<Graph> placeholder;
	Graph connectedComponent;
	std::queue<Node> unprocessedNodes;

	Node startNode = graph.getNode(0);
	unprocessedNodes.push(startNode);
	connectedComponent.addNode(startNode);

	breadthFirstSearch(graph, connectedComponent, unprocessedNodes);

	return placeholder;
}

bool unvisitedNeighboursExist(std::unordered_map<int, Node> visitedNodes, std::vector<int> neighboursOfcurrentNode);
int getUnvisitedNeighbourID(std::unordered_map<int, Node> visitedNodes, std::vector<int> neighboursOfcurrentNode);

void Algorithm::breadthFirstSearch(Graph graph, Graph &connectedComponent, std::queue<Node> &unprocessedNodes) {
	if (!unprocessedNodes.empty()) {
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
			breadthFirstSearch(graph, connectedComponent, unprocessedNodes);
		}
	}
}

bool unvisitedNeighboursExist(std::unordered_map<int, Node> visitedNodes, std::vector<int> neighboursIDsOfcurrentNode) {
	bool allNeighboursExist = true;
	int neighbourID = 0;

	for (size_t i = 0; i < neighboursIDsOfcurrentNode.size(); i++) {
		neighbourID = neighboursIDsOfcurrentNode[i];

		if (visitedNodes.find(neighbourID) == visitedNodes.end()) {
			allNeighboursExist = false;
		}
	}
	return allNeighboursExist;


}

int getUnvisitedNeighbourID(std::unordered_map<int, Node> visitedNodes, std::vector<int> neighboursOfcurrentNode) {
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