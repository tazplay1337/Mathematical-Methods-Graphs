#pragma once
#include "Graph.h"
#include "Node.h"
#include <algorithm>
using namespace std;

Graph::Graph(){}

Graph::~Graph(void) {}

void Graph::addNode(Node newNode) {
	//if (Knoten noch nicht da) TODO
	this->nodes.insert({ newNode.getID(), newNode });
}

void Graph::addEdge(Edge newEdge) {
	//if (Knoten noch nicht da) TODO
	this->edges.push_back(newEdge);
}

bool Graph::isEmpty() {
	//TODO
	return false;
}

Node Graph::getNode(int id) {
	if (nodes.find(id) == nodes.end()){
		return NULL;
	}
	else{
		return nodes[id];
	}
}

bool Graph::containNode(int id) {
	if (nodes.find(id) == nodes.end()) {
		return false;
	}
	else {
		return true;
	}
}

std::vector<int> Graph::getNeighboursID(Node node) {
	std::vector<int> neighbours;
	Edge edge;

	for (size_t i = 0; i < edges.size(); i++) {
		edge = edges[i];
		if (edge.getNodeV1ID() == node.getID()) {
			neighbours.push_back(edge.getNodeV2ID());
		}
		else if (edge.getNodeV2ID() == node.getID()) {
			neighbours.push_back(edge.getNodeV1ID());
		}
	}
	return neighbours;
}

std::unordered_map<int, Node> Graph::getNodes() {
	return this->nodes;
}

std::vector<int> Graph::getNodesID() {
	vector<int> nodesID;
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

