#pragma once
#include "Edge.h"
#include "Node.h"

Edge::Edge() {}

Edge::Edge(Node nodeV1, Node nodeV2) {
	this->nodeV1 = Node(nodeV1.getID());
	this->nodeV2 = Node(nodeV2.getID());
}

Node Edge::getNodeV1() {
	return this->nodeV1;
}

Node Edge::getNodeV2() {
	return this->nodeV2;
}

int Edge::getNodeV1ID() {
	return this->nodeV1.getID();
}

int Edge::getNodeV2ID() {
	return this->nodeV2.getID();
}