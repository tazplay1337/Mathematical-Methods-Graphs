#pragma once
#include <fstream>
#include <iostream>
#include "DataUtils.h"
#include "Algorithm.h"
#include "Graph.h"
#include "Node.h"

DataUtils::DataUtils() {}

void importAllNodes(std::ifstream &importGraph, Graph &intoGraph);
void importAllEdgesFromAdjList(std::ifstream &importGraph, Graph &intoGraph);

Graph DataUtils::importGraphFromAdjList(std::string fromPathOfFile) {
	std::ifstream importGraph(fromPathOfFile);
	Graph graph = Graph();

	if (importGraph) {
		importAllNodes(importGraph, graph);
		importAllEdgesFromAdjList(importGraph, graph);
	}
	return graph;
}

void importAllNodes(std::ifstream &importGraph, Graph &targetGraph) {
	Node newNode;
	std::string numOfAllNodesFromImportG;
	std::string edgeLineInAdjacentList;
	getline(importGraph, numOfAllNodesFromImportG);

	for (size_t id = 0; id < std::stoi(numOfAllNodesFromImportG); id++) {
		newNode = Node(id);
		targetGraph.addNode(newNode);
	}
}

int getFirstNodeIDFromAdjList(std::string &edgeLineInAdjacentList);
int getSecondNodeIDFromAdjList(std::string &edgeLineInAdjacentList);

void importAllEdgesFromAdjList(std::ifstream &importGraph, Graph &targetGraph) {
	int firstNodeID;
	int SecondNodeID;
	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		firstNodeID = getFirstNodeIDFromAdjList(edgeLineInAdjacentList);
		SecondNodeID = getSecondNodeIDFromAdjList(edgeLineInAdjacentList);
		targetGraph.addEdge(firstNodeID, SecondNodeID);
		targetGraph.updateNeighbour(firstNodeID, SecondNodeID);
	}
}

int getFirstNodeIDFromAdjList(std::string &edgeLineInAdjacentList) {
	const int START_INDEX_OF_FIRST_NODE = 0;
	int end_index_of_first_node = edgeLineInAdjacentList.find("\t");
	int firstNode = stoi(edgeLineInAdjacentList.substr(START_INDEX_OF_FIRST_NODE, end_index_of_first_node));
	return firstNode;	
}

int getSecondNodeIDFromAdjList(std::string &edgeLineInAdjacentList) {
	const int OFFSET_BEGINN_AFTER_TAB = 1;
	int start_index_of_second_node = edgeLineInAdjacentList.find("\t") + OFFSET_BEGINN_AFTER_TAB;
	int end_index_of_second_node = edgeLineInAdjacentList.find("\n");
	int secondNodeID = stoi(edgeLineInAdjacentList.substr(start_index_of_second_node, end_index_of_second_node));
	return secondNodeID;
}

void importAllEdgesFromAdjMatrix(std::ifstream &importGraph, Graph &intoGraph);

Graph DataUtils::importGraphFromAdjMatrix(std::string fromPathOfFile) {
	std::ifstream importGraph(fromPathOfFile);
	Graph graph = Graph();

	if (importGraph) {
		importAllNodes(importGraph, graph);
		importAllEdgesFromAdjMatrix(importGraph, graph);
	}
	return graph;
}

void removeTabs(std::string &line);

void importAllEdgesFromAdjMatrix(std::ifstream &importGraph, Graph &targetGraph) {
	const char IS_EDGE = '1';
	const int NEXT = 1;
	int firstNodeID = 0;
	int SecondNodeID = 0;
	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		removeTabs(edgeLineInAdjacentList);

		for (size_t SecondNodeID = firstNodeID; SecondNodeID < edgeLineInAdjacentList.length(); SecondNodeID++) {
			if (edgeLineInAdjacentList[SecondNodeID] == IS_EDGE) {
				targetGraph.addEdge(firstNodeID, SecondNodeID);
				targetGraph.updateNeighbour(firstNodeID, SecondNodeID);
			}
		}
		firstNodeID += NEXT;
	}
}

void removeTabs(std::string &line) {
	line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
}


