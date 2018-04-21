#pragma once
#include "Graph.h"
#include "Node.h"
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

std::vector<int> Graph::getNeighboursID(Node node) {

}

std::unordered_map<int, Node> Graph::getNodes() {

}

