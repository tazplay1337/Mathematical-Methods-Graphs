#pragma once
#include "Graph.h"
#include "Node.h"
#include <algorithm>
#include <iostream>

Graph::Graph(){}

Graph::~Graph(void) {}

void Graph::addNode(Node newNode) {
	bool nodeNotExist = true;	//= this->nodes.find(newNode.getID()) == this->nodes.end();

	if (nodeNotExist) {
		this->nodes.insert({ newNode.getID(), newNode });
	}
}

std::string createEdgeIndex(int nodeID1, int nodeID2);

void Graph::addEdge(int nodeID1, int nodeID2) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2);
	bool edgeNotExist = this->edges.find(edgeIndex) == this->edges.end();

	if (edgeNotExist) {
		Edge newEdge = Edge(nodeID1, nodeID2);
		this->edges.insert({ edgeIndex, newEdge });
	}
} 

void Graph::addEdge(int nodeID1, int nodeID2, double weight) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2);
	bool edgeNotExist = true; //= this->edges.find(edgeIndex) == this->edges.end();

	if (edgeNotExist) {
		Edge newEdge = Edge(nodeID1, nodeID2, weight);
		this->edges.insert({ edgeIndex, newEdge });
	}
}

std::string createEdgeIndex(int nodeID1, int nodeID2) { //	int index1 = nodeID1 * size + nodeID2;
	bool isNodeID1LowerNodeID2 = nodeID1 < nodeID2 ? true : false;
	std::string index;

	if (isNodeID1LowerNodeID2) {
		index = std::to_string(nodeID1) + ":" + std::to_string(nodeID2);
	}
	else {
		index = std::to_string(nodeID2) + ":" + std::to_string(nodeID1);
	}
	return index;
}

Edge Graph::getEdge(int nodeID1, int nodeID2) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2);
	bool edgeExist = true; //= this->edges.find(edgeIndex) != this->edges.end();

	if (edgeExist) {
		return edges[edgeIndex];
	}
	else{
		return Edge();
	}
}

std::vector<Edge> Graph::getNodeEdges(int nodeID) {
	std::vector<Edge> edgesNode;
	Edge edge;
	Node node = nodes[nodeID];
	std::vector<int> neighbours = node.getNeigbourIDs();

	for (size_t i = 0; i < neighbours.size(); i++) {
		edge = getEdge(node.getID(), neighbours[i]);
		edgesNode.push_back(edge);
	}
	return edgesNode;
}


void Graph::updateNeighbour(int firstNodeID, int secondNodeID) {
	this->nodes[firstNodeID].pushNeigbourID(secondNodeID);
	this->nodes[secondNodeID].pushNeigbourID(firstNodeID);	// TODO: If Graph ungerichtet else nicht
}

bool Graph::isEmpty() {
	//TODO
	return false;
}

Node Graph::getNode(int id) {
	bool nodeNotExist = nodes.find(id) == nodes.end();

	if (nodeNotExist){
		return NULL;
	}
	else{
		return nodes[id];
	}
}

bool Graph::nodeExist(int id) {
	bool nodeNotExist = nodes.find(id) == nodes.end();

	if (nodeNotExist) {
		return false;
	}
	else {
		return true;
	}
}

std::vector<int> Graph::getNeighboursID(int nodeID) {
	return this->nodes[nodeID].getNeigbourIDs();
}

std::unordered_map<int, Node> Graph::getNodes() {
	return this->nodes;
}

std::vector<int> Graph::getNodesID() {
	std::vector<int> nodesID;
	std::unordered_map<int, Node>::iterator node = nodes.begin();

	while (node != nodes.end()){
		nodesID.push_back(node->first);
		node++;
	}
	return nodesID;
}

int Graph::sizeEdges() {
	return this->edges.size();
}

int Graph::sizeNodes() {
	return this->nodes.size();
}

double Graph::totalCost() {
	Edge edge;
	double totalCost = 0;

	for (auto const& p : edges) {
		edge = p.second;
		totalCost += edge.getWeight();
	}
	return totalCost;
}

void Graph::printEdges() {
	Edge edge;
	int counter = 1;

	for (auto const& p : edges) {
		edge = p.second;
		std::cout << "Kante " << counter 
				  << ": von " << edge.getNodeIDV1() 
				  << " nach " << edge.getNodeIDV2() 
				  << " mit Gewicht " << edge.getWeight() << std::endl;
		counter += 1;
	}
}

void Graph::copyEdgesInVector(std::vector<Edge> &vectorForEdges) {
	Edge edge;
	double weight = 0;
	
	for (auto const& p : edges) {
		edge = p.second;
		vectorForEdges.push_back(edge);
	}
}







/*
void Graph::addEdge(int nodeID1, int nodeID2, double weight) {
	bool edgeNotExist = true;

	if (edgeNotExist) {
		this->edges[nodeID1].push_back(std::make_pair(nodeID2, weight));
		this->edges[nodeID2].push_back(std::make_pair(nodeID1, weight));
	}
}
*/

/*
std::string createEdgeIndex(int nodeID1, int nodeID2) {
	bool isnodeID1LowerNodeID2 = nodeID1<nodeID2 ? true : false;
	std::string index;

	if (isnodeID1LowerNodeID2) {
		index = std::to_string(nodeID1) + ":" + std::to_string(nodeID2);
	}
	else {
		index = std::to_string(nodeID2) + ":" + std::to_string(nodeID1);
	}
	return index;
}*/

/*
std::for_each(edges.begin(), edges.end(),
[](const std::unordered_map<std::string, Edge>::value_type& p){
std::cout << p.first << " => " << std::endl;
//		ege = p.second;
std::cout << ege.getWeight() << " => " << std::endl;
});*/