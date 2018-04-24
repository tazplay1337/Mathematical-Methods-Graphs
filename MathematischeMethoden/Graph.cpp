#pragma once
#include "Graph.h"
#include "Node.h"
#include <algorithm>

Graph::Graph(){}

Graph::~Graph(void) {}

void Graph::addNode(Node newNode) {
	bool nodeNotExist = this->nodes.find(newNode.getID()) == this->nodes.end();

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

bool Graph::containNode(int id) {
	bool nodeNotExist = nodes.find(id) == nodes.end();

	if (nodeNotExist) {
		return false;
	}
	else {
		return true;
	}
}

std::vector<int> Graph::getNeighboursID(Node node) {
	std::vector<int> neighbours;
/*	Edge edge;

	for (size_t i = 0; i < edges.size(); i++) {
		edge = edges[i];
		if (edge.getNodeV1ID() == node.getID()) {
			neighbours.push_back(edge.getNodeV2ID());
		}
		else if (edge.getNodeV2ID() == node.getID()) {
			neighbours.push_back(edge.getNodeV1ID());
		}
	} */
	return neighbours;
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

int Graph::edgesSize() {
	return edges.size();
}

int Graph::nodesSize() {
	return nodes.size();
}

