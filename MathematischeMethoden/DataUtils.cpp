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
int getFirstNodeIDFrom(std::string &edgeLineInAdjacentList);
int getSecondNodeIDFrom(std::string &edgeLineInAdjacentList);

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
	Node newNode;
	std::string numOfAllNodesFromImportG;
	std::string edgeLineInAdjacentList;
	getline(importGraph, numOfAllNodesFromImportG);

	for (int id = 0; id < std::stoi(numOfAllNodesFromImportG); id++) {
		newNode = Node(id);
		targetGraph.addNode(newNode);
	}
}

void importAllEdgesFrom(std::ifstream &importGraph, Graph &targetGraph) {
	int firstNodeID;
	int SecondNodeID;
	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		firstNodeID = getFirstNodeIDFrom(edgeLineInAdjacentList);
		SecondNodeID = getSecondNodeIDFrom(edgeLineInAdjacentList);
		targetGraph.addEdge(firstNodeID, SecondNodeID);
	}
}

int getFirstNodeIDFrom(std::string &edgeLineInAdjacentList) {	
	const int START_INDEX_OF_FIRST_NODE = 0;
	int end_index_of_first_node = edgeLineInAdjacentList.find("\t");
	int firstNode = stoi(edgeLineInAdjacentList.substr(START_INDEX_OF_FIRST_NODE, end_index_of_first_node));
	return firstNode;	
}

int getSecondNodeIDFrom(std::string &edgeLineInAdjacentList) {
	const int OFFSET_BEGINN_AFTER_TAB = 1;
	int start_index_of_second_node = edgeLineInAdjacentList.find("\t") + OFFSET_BEGINN_AFTER_TAB;
	int end_index_of_second_node = edgeLineInAdjacentList.find("\n");
	int secondNodeID = stoi(edgeLineInAdjacentList.substr(start_index_of_second_node, end_index_of_second_node));
	return secondNodeID;
}

Graph DataUtils::importGraphFromAdjMatrix(std::string fromPathOfFile) {
	// TODO
	return Graph();
}


