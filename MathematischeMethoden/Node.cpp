#pragma once
#include "Node.h"

Node::Node(){}

Node::Node(int id){
	this->id = id;
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