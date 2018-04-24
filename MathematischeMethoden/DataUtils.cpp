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
Node getFirstNodeFrom(std::string &edgeLineInAdjacentList);
Node getSecondNodeFrom(std::string &edgeLineInAdjacentList);

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

void importAllEdgesFrom(std::ifstream &importGraph, Graph &targetGraph) {
	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		Node firstNode = getFirstNodeFrom(edgeLineInAdjacentList);
		Node SecondNode = getSecondNodeFrom(edgeLineInAdjacentList);
		Edge newEdge = Edge(firstNode.getID(), SecondNode.getID());
		// Node firstNode = Graph.getNode(nodeId)
		// Node secondNode = Graph.getNode(node2Id)
		//FirstNode.Edge[] = newEdge
		targetGraph.addEdge(newEdge);
	}
}

Node getFirstNodeFrom(std::string &edgeLineInAdjacentList) {	
	const int START_INDEX_OF_FIRST_NODE = 0;
	int end_index_of_first_node = edgeLineInAdjacentList.find("\t");
	int firstNode = stoi(edgeLineInAdjacentList.substr(START_INDEX_OF_FIRST_NODE, end_index_of_first_node));
	return Node(firstNode);	
}

Node getSecondNodeFrom(std::string &edgeLineInAdjacentList) {
	const int OFFSET_BEGINN_AFTER_TAB = 1;
	int start_index_of_second_node = edgeLineInAdjacentList.find("\t") + OFFSET_BEGINN_AFTER_TAB;
	int end_index_of_second_node = edgeLineInAdjacentList.find("\n");
	int secondNode = stoi(edgeLineInAdjacentList.substr(start_index_of_second_node, end_index_of_second_node));
	return Node(secondNode);
}

Graph DataUtils::importGraphFromAdjMatrix(std::string fromPathOfFile) {
	// TODO
	return Graph();
}


