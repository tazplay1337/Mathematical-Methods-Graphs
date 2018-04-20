#pragma once
#include "Graph.h"
#include "Node.h"
using namespace std;

Graph::Graph(){}

Graph::Graph(AdjencyList initialAdjList) {
	this->adjList = initialAdjList;
}

Graph::~Graph(void) {}

void Graph::insertNode(int newNode) {
//	this->adjList.insert({ "First", 1 });;

	//newNodeY = Node(newNodeID);
	//if (Knoten noch nicht drin)
	//NodeListe.push(newNodeY);
}

void Graph::insertEdge(int v1, int v2) {
//	this->adjList.v1.insert(v2);
}