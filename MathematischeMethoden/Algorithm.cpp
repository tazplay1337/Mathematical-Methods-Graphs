#pragma once
#include "Algorithm.h"

Algorithm::Algorithm() {};

bool checkAllNeighborAreVisited();

std::vector<Edge> Algorithm::breadthFirstSearch(std::vector<Node> &L1, std::vector<Node> &L2, std::vector<Edge> &E0) {
	if (L1.empty) {
		return E0;
	}
	else {
		Node node = L1.begin;
		
		if (checkAllNeighborAreVisited());
	}
}

std::vector<Edge> Algorithm::depthFirstSearch() {

}


std::vector<Graph> Algorithm::getConnectedComponentWithBFS(Graph graph) {
//	std::vector<Graph> connectedComponentsOfG;
	std::vector<Edge> edgesOfConnectedComponent;
	std::vector<Node> listOfRechedNodesInG;
	std::vector<Node> listOfVisitedNodesInG;

	Node startNode = graph.getNode(0);
	listOfRechedNodesInG.push_back(startNode);
	listOfVisitedNodesInG.push_back(startNode);

	breadthFirstSearch(listOfRechedNodesInG, listOfVisitedNodesInG, edgesOfConnectedComponent);

	return connectedComponentsOfG;
} 

/*
std::vector<Edge> Algorithm::getConnectedComponentWithDFS(Graph graph) {

}*/