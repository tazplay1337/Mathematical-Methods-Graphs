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

Graph DataUtils::importDirectedGraphFromAdjList(std::string fromPathOfFile) {
	std::ifstream importGraph(fromPathOfFile);
	Graph graph = Graph(true);

	if (importGraph) {
		importAllNodes(importGraph, graph);
		importAllEdgesFromAdjList(importGraph, graph);
	}
	return graph;
}

void importAllCapacityEdgesFromAdjList(std::ifstream &importGraph, Graph &intoGraph);

Graph DataUtils::importDirectedGraphWithCapacityFromAdjList(std::string fromPathOfFile) {
	std::ifstream importGraph(fromPathOfFile);
	Graph graph = Graph(true);

	if (importGraph) {
		importAllNodes(importGraph, graph);
		importAllCapacityEdgesFromAdjList(importGraph, graph);
	}
	return graph;
}

void importAllBalanceNodes(std::ifstream &importGraph, Graph &intoGraph);
void importAllCostCapaEdgesFromAdjList(std::ifstream &importGraph, Graph &intoGraph);

Graph DataUtils::importDirectedBalanceGraphFromAdjList(std::string fromPathOfFile) {
	std::ifstream importGraph(fromPathOfFile);
	Graph graph = Graph(true);

	if (importGraph) {
		importAllBalanceNodes(importGraph, graph);
		importAllCostCapaEdgesFromAdjList(importGraph, graph);
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
		}
		else {
			targetGraph.addEdge(firstNodeID, SecondNodeID);
		}
	}
}

void importAllCapacityEdgesFromAdjList(std::ifstream &importGraph, Graph &targetGraph) {
	int firstNodeID = 0;
	int SecondNodeID = 0;
	double capacity = 0;

	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		firstNodeID = getFirstNodeIDFromAdjList(edgeLineInAdjacentList);
		SecondNodeID = getSecondNodeIDFromAdjList(edgeLineInAdjacentList);
		setWeightIfExist(edgeLineInAdjacentList, capacity);
		targetGraph.addEdge(firstNodeID, SecondNodeID, 0, capacity);
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
			}
		}
		firstNodeID += 1;
	}
}

void removeTabs(std::string &line) {
	line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
}


//NEU
void importAllBalanceNodes(std::ifstream &importGraph, Graph &targetGraph) {
	Node newNode;
	std::string numOfAllNodesFromImportG;
	std::string balanceLineInAdjacentList;
	double balance = 0.0;
	getline(importGraph, numOfAllNodesFromImportG);

	for (size_t id = 0; id < std::stoi(numOfAllNodesFromImportG); id++) {
		getline(importGraph, balanceLineInAdjacentList);
		balance = stoi(balanceLineInAdjacentList);
		newNode = Node(id, balance);
		targetGraph.addNode(newNode);
	}
}

bool setCapacityIfExist(std::string &edgeLineInAdjacentList, double &capacity);

void importAllCostCapaEdgesFromAdjList(std::ifstream &importGraph, Graph &targetGraph) {
	int firstNodeID = 0;
	int SecondNodeID = 0;
	double cost = 0.0;
	double capacity = 0.0;

	std::string edgeLineInAdjacentList;

	while (getline(importGraph, edgeLineInAdjacentList)) {
		firstNodeID = getFirstNodeIDFromAdjList(edgeLineInAdjacentList);
		SecondNodeID = getSecondNodeIDFromAdjList(edgeLineInAdjacentList);
		setWeightIfExist(edgeLineInAdjacentList, cost);
		setCapacityIfExist(edgeLineInAdjacentList, capacity);
		targetGraph.addEdge(firstNodeID, SecondNodeID, cost, capacity);

	}
}

int findThirdTab(std::string &edgeLineInAdjacentList);

bool setCapacityIfExist(std::string &edgeLineInAdjacentList, double &capacity) {
	const int cOffsetBeginnAfterTab = 1;
	int indexOfThirdTab = findThirdTab(edgeLineInAdjacentList);
	bool capacitytExist = false;

	if (indexOfThirdTab != std::string::npos) {
		int startIndexOfWeight = indexOfThirdTab + cOffsetBeginnAfterTab;
		int endIndexOfWeight = edgeLineInAdjacentList.size() - 1;
		capacity = stod(edgeLineInAdjacentList.substr(startIndexOfWeight, endIndexOfWeight));
		capacitytExist = true;
	}
	return capacitytExist;
}

int findThirdTab(std::string &edgeLineInAdjacentList) {
	const int cOffsetBeginnAfterTab = 1;
	size_t firstTab = edgeLineInAdjacentList.find("\t");
	size_t secondTab = std::string::npos;
	size_t thirdTab = std::string::npos;

	if (firstTab != std::string::npos) {
		secondTab = edgeLineInAdjacentList.find("\t", firstTab + cOffsetBeginnAfterTab);
	}

	if (secondTab != std::string::npos) {
		thirdTab = edgeLineInAdjacentList.find("\t", secondTab + cOffsetBeginnAfterTab);
	}
	return thirdTab;
}