#pragma once
#include "Edge.h"
#include "Node.h"

const int cMyNULL = -99999;

Edge::Edge() {
	this->beInExistence = false;
	this->nodeIDV1 = cMyNULL;
	this->nodeIDV2 = cMyNULL;
	this->hasWeigt = false;
	this->weight = cMyNULL;
	this->capacity = cMyNULL;
	this->flow = 0;
}

Edge::Edge(int nodeIDV1, int nodeIDV2) {
	this->beInExistence = true;
	this->hasWeigt = false;
	this->nodeIDV1 = nodeIDV1;
	this->nodeIDV2 = nodeIDV2;
	this->weight = cMyNULL;
	this->capacity = cMyNULL;
	this->flow = 0;
}

Edge::Edge(int nodeIDV1, int nodeIDV2, double weight) {
	this->beInExistence = true;
	this->nodeIDV1 = nodeIDV1;
	this->nodeIDV2 = nodeIDV2;
	this->hasWeigt = true;
	this->weight = weight;
	this->capacity = cMyNULL;
	this->flow = 0;
}

Edge::Edge(int nodeIDV1, int nodeIDV2, double cost, double capacity) {
	this->beInExistence = true;
	this->nodeIDV1 = nodeIDV1;
	this->nodeIDV2 = nodeIDV2;
	this->hasWeigt = true;
	this->weight = cost;
	this->capacity = capacity;
	this->flow = 0;
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

void Edge::setWeight(double weight) {
	this->weight = weight;
}

void Edge::addWeight(double weight) {
	this->weight += weight;
}

double Edge::getCapacity() {
	return this->capacity;
}

void Edge::setCapacity(double capacity) {
	this->capacity = capacity;
}

void Edge::addCapacity(double capacity) {
	this->capacity += capacity;
}

double Edge::getFlow() {
	return this->flow;
}

void Edge::setFlow(double flow) {
	this->flow = flow;
}

void Edge::addFlow(double flow) {
	this->flow += flow;
}

bool Edge::exist() {
	return this->beInExistence;
}