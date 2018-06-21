#pragma once
#include "Node.h"

Node::Node(){}

Node::Node(int id){
	this->id = id;
	this->balance = 0;
}

Node::Node(int id, double balance) {
	this->id = id;
	this->balance = balance;
}

int Node::getID() {
	return this->id;
}

void Node::pushNeigbourID(int newNeigbourID) {
	bool isNewNeigbourID = true;

	for (int i = 0; i < neigbourIDs.size(); i++) {
		if (newNeigbourID == neigbourIDs[i]) {
			newNeigbourID = false;
		}
	}

	if (isNewNeigbourID) {
		neigbourIDs.push_back(newNeigbourID);
	}
}

void Node::removeNeigbour(double nodeID) {

	for (int i = 0; i < this->neigbourIDs.size(); i++) {
		if (this->neigbourIDs[i] == nodeID) {
			this->neigbourIDs.erase(neigbourIDs.begin() + i);
		}
	}
}

std::vector<int> Node::getNeigbourIDs() {
	return this->neigbourIDs;
}

double Node::getBalance() {
	return this->balance;
}

void Node::setBalance(double balance) {
	this->balance = balance;
}