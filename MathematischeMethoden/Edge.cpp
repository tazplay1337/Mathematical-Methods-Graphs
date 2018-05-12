#pragma once
#include "Edge.h"
#include "Node.h"

Edge::Edge() {
	const int cMyNULL = -99999;
	this->beInExistence = false;
	this->nodeIDV1 = cMyNULL;
	this->nodeIDV2 = cMyNULL;
	this->hasWeigt = false;
	this->weight = cMyNULL;
}

Edge::Edge(int nodeIDV1, int nodeIDV2) {
	this->beInExistence = true;
	this->hasWeigt = false;
	this->nodeIDV1 = nodeIDV1;
	this->nodeIDV2 = nodeIDV2;
	this->weight = 0;
}

Edge::Edge(int nodeIDV1, int nodeIDV2, double weight) {
	this->beInExistence = true;
	this->nodeIDV1 = nodeIDV1;
	this->nodeIDV2 = nodeIDV2;
	this->hasWeigt = true;
	this->weight = weight;
}

int Edge::getNodeIDV1() {
	return this->nodeIDV1;
}

int Edge::getNodeIDV2() {
	return this->nodeIDV2;
}

double Edge::getWeight() {
	return this->weight;
}

bool Edge::exist() {
	return this->beInExistence;
}