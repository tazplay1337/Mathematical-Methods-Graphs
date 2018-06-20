#pragma once
#include "Node.h"

Node::Node(){}

Node::Node(int id){
	this->id = id;
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

std::vector<int> Node::getNeigbourIDs() {
	return this->neigbourIDs;
}

double Node::getBalance() {
	return this->balance;
}

void Node::setBalance(double balance) {
	this->balance = balance;
}