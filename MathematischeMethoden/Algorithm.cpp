#pragma once
#include "Algorithm.h"


Algorithm::Algorithm() {};

bool checkNeighboursIfVisitedOf(Node node);
Node getUnvisitedNeighbourOf(Node node);
Edge createEdge(Node nodeV1, Node nodeV2);

std::vector<Edge> Algorithm::breadthFirstSearch(Graph graph,
												std::queue<Node> &listOfRechedNodesInG, 
												std::queue<Node> &listOfVisitedNodesInG, 
												std::vector<Edge> &edgesOfConnectedComponent) {
	if (listOfRechedNodesInG.empty()) {
		return edgesOfConnectedComponent;
	}
	else {
		Node currentNode = listOfRechedNodesInG.pop(); //Syntax noch nicht ganz richtig
		
		if (checkNeighboursIfVisitedOf(currentNode)) {
			//Delete currentNode from listOfRechedNodesInG;
		}
		else {
			Node unvisitedNode = getUnvisitedNeighbourOf(currentNode);
			listOfRechedNodesInG.push(unvisitedNode);
			listOfVisitedNodesInG.push(unvisitedNode);
			Edge newEdge = createEdge(currentNode, unvisitedNode);
			edgesOfConnectedComponent.push_back(newEdge);
			breadthFirstSearch(graph, listOfRechedNodesInG, listOfVisitedNodesInG, edgesOfConnectedComponent);
		}
	}
}

bool checkNeighboursIfVisitedOf(Node node) {

	return false;
}

Node getUnvisitedNeighbourOf(Node node) {
	return Node();
}

Edge createEdge(Node nodeV1, Node nodeV2) {
	return Edge();
}

std::vector<Edge> Algorithm::depthFirstSearch() {

}


std::vector<Graph> Algorithm::getConnectedComponentWithBFS(Graph graph) {
	std::vector<Graph> connectedComponentsOfG;
	std::vector<Edge> edgesOfConnectedComponent;
	std::queue<Node> unprocessedNodesOfG;
	std::queue<Node> visitedNodesOfG;

	Node startNode = graph.getNode(0);
	unprocessedNodesOfG.push(startNode);
	visitedNodesOfG.push(startNode);

	breadthFirstSearch(graph, unprocessedNodesOfG, visitedNodesOfG, edgesOfConnectedComponent);

	return connectedComponentsOfG;
} 

/*
std::vector<Edge> Algorithm::getConnectedComponentWithDFS(Graph graph) {

}*/