#pragma once
#include "Graph.h"
#include "Node.h"
#include <algorithm>
#include <iostream>

Graph::Graph(){
	this->edgesDirected = false;
}

Graph::Graph(bool edgesDirected) {
	this->edgesDirected = edgesDirected;
}

Graph::Graph(bool edgesDirected, std::unordered_map<int, Node> nodes, std::unordered_map<std::string, Edge> edges, double valFlow) {
	this->edgesDirected = edgesDirected;
	this->edges = edges;
	this->valTotalFlow = valFlow;
	this->nodes = nodes;
}

Graph::~Graph(void) {}

void Graph::addNode(Node newNode) {
	bool nodeNotExist = true;	//= this->nodes.find(newNode.getID()) == this->nodes.end();

	if (nodeNotExist) {
		this->nodes.insert({ newNode.getID(), newNode });
	}
}

std::string createEdgeIndex(int nodeID1, int nodeID2, bool edgesDirected);

void Graph::addEdge(int nodeID1, int nodeID2) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2, this->edgesDirected);
	bool edgeNotExist = this->edges.find(edgeIndex) == this->edges.end();

	if (edgeNotExist) {
		Edge newEdge = Edge(nodeID1, nodeID2);
		this->edges.insert({ edgeIndex, newEdge });
		updateNeighbour(nodeID1, nodeID2);
	}
} 

void Graph::addEdge(int nodeID1, int nodeID2, double weight) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2, this->edgesDirected);
	bool edgeNotExist = true; //= this->edges.find(edgeIndex) == this->edges.end();

	if (edgeNotExist) {
		Edge newEdge = Edge(nodeID1, nodeID2, weight);
		this->edges.insert({ edgeIndex, newEdge });
		updateNeighbour(nodeID1, nodeID2);
	}
}

void Graph::addEdge(int nodeID1, int nodeID2, double weight, double capacity) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2, this->edgesDirected);
	bool edgeNotExist = true; //= this->edges.find(edgeIndex) == this->edges.end();

	if (edgeNotExist) {
		Edge newEdge = Edge(nodeID1, nodeID2, weight, capacity);
		this->edges.insert({ edgeIndex, newEdge });
		updateNeighbour(nodeID1, nodeID2);
	}
}

std::string createEdgeIndex(int nodeID1, int nodeID2, bool edgesDirected) {
	bool isNodeID1LowerNodeID2 = nodeID1 < nodeID2 ? true : false;
	std::string index;

	if (edgesDirected || isNodeID1LowerNodeID2) {
		index = std::to_string(nodeID1) + ":" + std::to_string(nodeID2);
	}
	else {
		index = std::to_string(nodeID2) + ":" + std::to_string(nodeID1);
	}
	return index;
}

Edge Graph::getEdge(int nodeID1, int nodeID2) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2, this->edgesDirected);
	bool edgeExist = true; //= this->edges.find(edgeIndex) != this->edges.end();

	if (edgeExist) {
		return edges[edgeIndex];
	}
	else{
		return Edge();
	}
}

double Graph::getEdgeCost(int nodeID1, int nodeID2) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2, this->edgesDirected);
	bool edgeExist = true; //= this->edges.find(edgeIndex) != this->edges.end();

	if (edgeExist) {
		return edges[edgeIndex].getWeight();
	}
	else {
		return std::numeric_limits<double>::max();
	}
}

double Graph::getEdgeCapacity(int nodeID1, int nodeID2) {
	std::string edgeIndex = createEdgeIndex(nodeID1, nodeID2, this->edgesDirected);
	bool edgeExist = true; //= this->edges.find(edgeIndex) != this->edges.end();

	if (edgeExist) {
		return edges[edgeIndex].getCapacity();
	}
	else {
		return std::numeric_limits<double>::max();
	}
}

void Graph::setEdgeFlow(std::string index, double flow) {
	bool edgeExist = true; //= this->edges.find(edgeIndex) != this->edges.end();

	if (edgeExist) {
		edges[index].setFlow(flow);
	}
}

std::vector<Edge> Graph::getNodeEdges(int nodeID) {
	std::vector<Edge> edgesNode;
	Edge edge;
	Node node = nodes[nodeID];
	std::vector<int> neighbours = node.getNeigbourIDs();

	for (size_t i = 0; i < neighbours.size(); i++) {
		edge = getEdge(node.getID(), neighbours[i]);
		edgesNode.push_back(edge);
	}
	return edgesNode;
}


void Graph::updateNeighbour(int firstNodeID, int secondNodeID) {
	this->nodes[firstNodeID].pushNeigbourID(secondNodeID);
	
	if(!this->edgesDirected) {
		this->nodes[secondNodeID].pushNeigbourID(firstNodeID);
	}
}

bool Graph::isEmpty() {
	//TODO
	return false;
}

bool Graph::isDirected() {
	return this->edgesDirected;
}

Node Graph::getNode(int id) {
	bool nodeNotExist = nodes.find(id) == nodes.end();

	if (nodeNotExist){
		return NULL;
	}
	else{
		return nodes[id];
	}
}

double  Graph::getNodeBalance(int id) {
	bool nodeNotExist = nodes.find(id) == nodes.end();

	if (nodeNotExist) {
		return 0;
	}
	else {
		return nodes[id].getBalance();
	}
}

bool Graph::nodeExist(int id) {
	bool nodeNotExist = nodes.find(id) == nodes.end();

	if (nodeNotExist) {
		return false;
	}
	else {
		return true;
	}
}

std::vector<int> Graph::getNeighboursID(int nodeID) {
	return this->nodes[nodeID].getNeigbourIDs();
}

std::unordered_map<int, Node> Graph::getNodes() {
	return this->nodes;
}

std::unordered_map<std::string, Edge> Graph::getEdges() {
	return this->edges;
}

std::vector<int> Graph::getNodesID() {
	std::vector<int> nodesID;
	std::unordered_map<int, Node>::iterator node = nodes.begin();

	while (node != nodes.end()){
		nodesID.push_back(node->first);
		node++;
	}
	return nodesID;
}

int Graph::sizeEdges() {
	return this->edges.size();
}

int Graph::sizeNodes() {
	return this->nodes.size();
}

double Graph::totalCost() {
	Edge edge;
	double totalCost = 0;

	for (auto const& p : edges) {
		edge = p.second;
		totalCost += edge.getWeight();
	}
	return totalCost;
}

void Graph::printEdges() {
	Edge edge;
	int counter = 1;

	for (auto const& p : edges) {
		if (counter > 30) {
			break;
		}
		edge = p.second;
		std::cout << "Kante " << counter 
				  << ": von " << edge.getNodeIDV1() 
				  << " nach " << edge.getNodeIDV2() 
				  << " mit Gewicht " << edge.getWeight() << std::endl;
		counter += 1;
	}
}

void Graph::copyEdgesInVector(std::vector<Edge> &vectorForEdges) {
	Edge edge;
	double weight = 0;
	
	for (auto const& p : edges) {
		edge = p.second;
		vectorForEdges.push_back(edge);
	}
}

bool Graph::hasNegativeCostEdge() {
	Edge edge;
	bool hasNegativeCost = false;

	for (auto const& p : edges) {
		edge = p.second;
		
		if (edge.getWeight() < 0) {
			hasNegativeCost = true;
			return hasNegativeCost;
		}
	}
	return hasNegativeCost;
}

double Graph::getValTotalFlow() {
	return this->valTotalFlow;
}

void Graph::deleteNode(int nodeID) {
	std::vector<int> neigbourIDs = this->nodes[nodeID].getNeigbourIDs();
	std::string edgeIndexForward;
	std::string edgeIndexBackward;
	bool edgeIndexForwardExist;
	bool edgeIndexBackwardExist;

	for (int i = 0; i < neigbourIDs.size(); i++) {
		edgeIndexForward = createEdgeIndex(nodeID, neigbourIDs[i], this->edgesDirected);
		edgeIndexBackward = createEdgeIndex(neigbourIDs[i], nodeID, this->edgesDirected);

		edgeIndexForwardExist = this->edges.find(edgeIndexForward) != this->edges.end();
		edgeIndexBackwardExist = this->edges.find(edgeIndexBackward) != this->edges.end();

		if (edgeIndexForwardExist) {
			this->edges.erase(edgeIndexForward);
		}
		else if(edgeIndexBackwardExist) {
			this->edges.erase(edgeIndexBackward);
			this->nodes[neigbourIDs[i]].removeNeigbour(nodeID);
		}
	}

	for (int nodeID2 = 0; nodeID2 < nodes.size(); nodeID2++) {
		edgeIndexBackward = createEdgeIndex(nodeID2, nodeID, this->edgesDirected);

		edgeIndexBackwardExist = this->edges.find(edgeIndexBackward) != this->edges.end();

		if (edgeIndexBackwardExist) {
			this->edges.erase(edgeIndexBackward);
			this->nodes[nodeID2].removeNeigbour(nodeID);
		}
	}

	this->nodes.erase(nodeID);
}








/*
void Graph::addEdge(int nodeID1, int nodeID2, double weight) {
	bool edgeNotExist = true;

	if (edgeNotExist) {
		this->edges[nodeID1].push_back(std::make_pair(nodeID2, weight));
		this->edges[nodeID2].push_back(std::make_pair(nodeID1, weight));
	}
}
*/

/*
std::string createEdgeIndex(int nodeID1, int nodeID2) {
	bool isnodeID1LowerNodeID2 = nodeID1<nodeID2 ? true : false;
	std::string index;

	if (isnodeID1LowerNodeID2) {
		index = std::to_string(nodeID1) + ":" + std::to_string(nodeID2);
	}
	else {
		index = std::to_string(nodeID2) + ":" + std::to_string(nodeID1);
	}
	return index;
}*/

/*
std::for_each(edges.begin(), edges.end(),
[](const std::unordered_map<std::string, Edge>::value_type& p){
std::cout << p.first << " => " << std::endl;
//		ege = p.second;
std::cout << ege.getWeight() << " => " << std::endl;
});*/