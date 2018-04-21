#pragma once
#include <fstream>
#include <iostream>
#include "DataUtils.h"
#include "Algorithm.h"
#include "Graph.h"
#include "Node.h"

DataUtils::DataUtils() {}

void importAllNodesFrom(std::ifstream &importGraph, Graph &intoGraph);
void importAllEdgesFrom(std::ifstream &importGraph, Graph &intoGraph);
Node getFirstNodeFrom(std::string edgeLineInAdjacentList);
Node getSecondNodeFrom(std::string edgeLineInAdjacentList);

Graph DataUtils::importGraphFromAdjList(std::string fromPathOfFile) {
	std::ifstream importGraph(fromPathOfFile);
	Graph graph = Graph();

	if (importGraph) {
		importAllNodesFrom(importGraph, graph);
		importAllEdgesFrom(importGraph, graph);
	}
	return graph;
}

void importAllNodesFrom(std::ifstream &importGraph, Graph &targetGraph) {
	std::string numOfAllNodesFromImportG;
	std::string edgeLineInAdjacentList;
	getline(importGraph, numOfAllNodesFromImportG);

	for (int id = 0; id < std::stoi(numOfAllNodesFromImportG); id++) {
		Node newNode = Node(id);
		targetGraph.addNode(newNode);
	}
}

Node getFirstNodeFrom(std::string edgeLineInAdjacentList) {	
	const int INDEX_OF_FIRST_NODE = 0;
	const int OFF_SET_CHAR_TO_INT = 48;
	int firstNode = (int)edgeLineInAdjacentList[INDEX_OF_FIRST_NODE] - OFF_SET_CHAR_TO_INT;
	return Node(firstNode);	
}

void importAllEdgesFrom(std::ifstream &importGraph, Graph &targetGraph) {
	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		Node firstNode = getFirstNodeFrom(edgeLineInAdjacentList);
		Node SecondNode = getSecondNodeFrom(edgeLineInAdjacentList);
		Edge newEdge = Edge(firstNode.getID(), SecondNode.getID());
		targetGraph.addEdge(newEdge);
	}
}

Node getSecondNodeFrom(std::string edgeLineInAdjacentList) {
	const int INDEX_OF_SECOND_NODE = 2;
	const int OFF_SET_CHAR_TO_INT = 48;
	int secondNode = (int)edgeLineInAdjacentList[INDEX_OF_SECOND_NODE] - OFF_SET_CHAR_TO_INT;
	return Node(secondNode);
}

Graph DataUtils::importGraphFromAdjMatrix(std::string fromPathOfFile) {
	// TODO
	return Graph();
}


