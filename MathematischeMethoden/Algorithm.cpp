#pragma once
#include "Algorithm.h"


Algorithm::Algorithm() {};

bool unvisitedNeighboursExistOf(Node node);
Node getUnvisitedNeighbourOf(Node node);

void Algorithm::breadthFirstSearch(Graph graph, Graph &connectedComponent, std::queue<Node> &unprocessedNodes) {
	if (!unprocessedNodes.empty()) {
		Node currentNode = unprocessedNodes.front();
		std::vector<int> neighboursOfcurrentNode = graph.getNeighboursID(currentNode);
		std::unordered_map<int, Node> visitedNodes = connectedComponent.getNodes();
		
		if (!unvisitedNeighboursExist(visitedNodes, neighboursOfcurrentNode)) {
			unprocessedNodes.pop();
		}
		else {
			Node unvisitedNode = getUnvisitedNeighbourOf(currentNode);
			unprocessedNodes.push(unvisitedNode);
			connectedComponent.addNode(unvisitedNode);
			Edge newEdge = Edge(currentNode, unvisitedNode);
			connectedComponent.addEdge(newEdge);
			breadthFirstSearch(graph, connectedComponent, unprocessedNodes);
		}
	}
}

bool unvisitedNeighboursExist(std::unordered_map<int, Node> visitedNodes, std::vector<int> neighboursOfcurrentNode) {

	return false;
}

Node getUnvisitedNeighbourOf(Node node) {
	return Node();
}
/*
std::vector<Edge> Algorithm::depthFirstSearch() {

}
*/

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

/*
std::vector<Edge> Algorithm::getConnectedComponentWithDFS(Graph graph) {

}*/