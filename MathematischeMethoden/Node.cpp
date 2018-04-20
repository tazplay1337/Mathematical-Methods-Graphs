#pragma once
#include "Node.h"

Node::Node(){}

Node::Node(int id){
	this->id = id;
}

int Node::getID() {
	return this->id;
}