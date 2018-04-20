#pragma once
#include "Graph.h"
#include "Node.h"
using namespace std;

Graph::Graph(){}

Graph::~Graph(void) {}

void Graph::insertNode(Node newNode) {
	//if (Knoten noch nicht da)
	this->allNodesOfG.insert({ newNode.getID(), newNode });
}

void Graph::insertEdge(Edge newEdge) {
	//if (Knoten noch nicht da)
	this->allEdgesOfG.insert(allEdgesOfG.begin(), newEdge);
}