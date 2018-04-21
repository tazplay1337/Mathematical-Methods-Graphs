#pragma once
#include "Graph.h"
#include "Node.h"
using namespace std;

Graph::Graph(){}

Graph::~Graph(void) {}

void Graph::insertNode(Node newNode) {
	//if (Knoten noch nicht da) TODO
	this->nodes.insert({ newNode.getID(), newNode });
}

void Graph::insertEdge(Edge newEdge) {
	//if (Knoten noch nicht da) TODO
	this->edges.push_back(newEdge);
}

bool Graph::isEmpty() {
	//TODO
	return false;
}

Node Graph::getNode(int id) {
	//if (Knoten noch nicht da) TODO
	Node node = this->nodes.find(id);
	return node;
}

