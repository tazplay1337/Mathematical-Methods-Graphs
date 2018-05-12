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
bool setWeightIfExist(std::string &edgeLineInAdjacentList, double &weight);

void importAllEdgesFromAdjList(std::ifstream &importGraph, Graph &targetGraph) {
	int firstNodeID = 0;
	int SecondNodeID = 0;
	bool isWeight = false;
	double weight = 0;

	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		firstNodeID = getFirstNodeIDFromAdjList(edgeLineInAdjacentList);
		SecondNodeID = getSecondNodeIDFromAdjList(edgeLineInAdjacentList);
		isWeight = setWeightIfExist(edgeLineInAdjacentList, weight);

		if (isWeight) {
			targetGraph.addEdge(firstNodeID, SecondNodeID, weight);
			targetGraph.updateNeighbour(firstNodeID, SecondNodeID);
		}
		else {
			targetGraph.addEdge(firstNodeID, SecondNodeID);
			targetGraph.updateNeighbour(firstNodeID, SecondNodeID);
		}
		
	}
}

int getFirstNodeIDFromAdjList(std::string &edgeLineInAdjacentList) {
	const int cStartIndexOfFirstNode = 0;
	int end_index_of_first_node = edgeLineInAdjacentList.find("\t");
	int firstNode = stoi(edgeLineInAdjacentList.substr(cStartIndexOfFirstNode, end_index_of_first_node));
	return firstNode;	
}

int getSecondNodeIDFromAdjList(std::string &edgeLineInAdjacentList) {
	const int cOffsetBeginnAfterTab = 1;
	int start_index_of_second_node = edgeLineInAdjacentList.find("\t") + cOffsetBeginnAfterTab;
	int end_index_of_second_node = edgeLineInAdjacentList.find("\n");
	int secondNodeID = stoi(edgeLineInAdjacentList.substr(start_index_of_second_node, end_index_of_second_node));
	return secondNodeID;
}

int findSecondTab(std::string &edgeLineInAdjacentList);

bool setWeightIfExist(std::string &edgeLineInAdjacentList, double &weight) {
	const int cOffsetBeginnAfterTab = 1;
	int indexOfSecondTab = findSecondTab(edgeLineInAdjacentList);
	bool weightExist = false;

	if (indexOfSecondTab != std::string::npos) {
		int startIndexOfWeight = indexOfSecondTab + cOffsetBeginnAfterTab;
		int endIndexOfWeight = edgeLineInAdjacentList.size() - 1;
		weight = stod(edgeLineInAdjacentList.substr(startIndexOfWeight, endIndexOfWeight));
		weightExist = true;
	}
	return weightExist;
}

int findSecondTab(std::string &edgeLineInAdjacentList) {
	const int cOffsetBeginnAfterTab = 1;
	size_t firstTab = edgeLineInAdjacentList.find("\t");
	size_t secondTab = std::string::npos;

	if (firstTab != std::string::npos) {
		secondTab = edgeLineInAdjacentList.find("\t", firstTab + cOffsetBeginnAfterTab);
	}
	return secondTab;
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
	const char cIsEdge = '1';
	int firstNodeID = 0;
	int SecondNodeID = 0;
	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		removeTabs(edgeLineInAdjacentList);

		for (size_t SecondNodeID = firstNodeID; SecondNodeID < edgeLineInAdjacentList.length(); SecondNodeID++) {
			if (edgeLineInAdjacentList[SecondNodeID] == cIsEdge) {
				targetGraph.addEdge(firstNodeID, SecondNodeID);
				targetGraph.updateNeighbour(firstNodeID, SecondNodeID);
			}
		}
		firstNodeID += 1;
	}
}

void removeTabs(std::string &line) {
	line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
}


