#pragma once
#include "Algorithm.h"
#include "DisjointSets.h"
#include <iostream>
#include <utility>
#include <queue>

Algorithm::Algorithm() {};

int totalNodesSize(std::vector<Graph> &graphs);
int findUnvisitedNodeID(std::vector<bool> &visitedNodes);
void setVisitedNodes(std::vector<bool> &visitedNodes, Graph &graph);

std::vector<Graph> Algorithm::getConnectedComponentWithBFS(Graph &graph) {
	Graph connectedComponent;
	std::vector<Graph> connectedComponentsOfGraph;
	int sizeGraph = graph.sizeNodes();
	int unvisitedNodeID = 0;
	std::vector<bool> visitedNodes(sizeGraph, false);

	while (totalNodesSize(connectedComponentsOfGraph) < sizeGraph) {
		unvisitedNodeID = findUnvisitedNodeID(visitedNodes); //starte vom letzten gefundenen
		connectedComponent = breadthFirstSearch(graph, unvisitedNodeID);
		connectedComponentsOfGraph.push_back(connectedComponent);
		setVisitedNodes(visitedNodes, connectedComponent);
	}
	return connectedComponentsOfGraph;
}

int totalNodesSize(std::vector<Graph> &graphs) {
	int totalSize = 0;

	for (size_t i = 0; i < graphs.size(); i++) {
		totalSize += graphs[i].sizeNodes();
	}
	return totalSize;
}

int findUnvisitedNodeID(std::vector<bool> &visitedNodes) {
	const int ALL_NODES_VISITED = -1;
	int nodeID = ALL_NODES_VISITED;
	bool isNodeVisited = true;

	for (size_t i = 0; isNodeVisited && i < visitedNodes.size(); i++) {
		isNodeVisited = visitedNodes[i];

		if (!isNodeVisited) {
			nodeID = i;
		}
	}
	return nodeID;
}

void setVisitedNodes(std::vector<bool> &visitedNodes, Graph &graph) {
	for (size_t i = 0; i < visitedNodes.size(); i++) {
		if (graph.nodeExist(i)) {
			visitedNodes[i] = true;
		}
	}
}

Graph Algorithm::breadthFirstSearch(Graph &graph, int startNodeID) {
	int currentNodeID = startNodeID;
	int neighbourNode = 0;
	Graph connectedComponent;
	std::queue<int> unprocessedNodes;
	std::vector<int> neighbourIDsOfcurrentNode;
	
	connectedComponent.addNode(currentNodeID);
	unprocessedNodes.push(currentNodeID);
	
	while (!unprocessedNodes.empty()) {
		currentNodeID = unprocessedNodes.front();
		unprocessedNodes.pop();
		neighbourIDsOfcurrentNode = graph.getNeighboursID(currentNodeID);

		for (size_t i = 0; i < neighbourIDsOfcurrentNode.size(); i++) {
			neighbourNode = neighbourIDsOfcurrentNode[i];

			if (!connectedComponent.nodeExist(neighbourNode)) {
				unprocessedNodes.push(neighbourNode);
				connectedComponent.addNode(neighbourNode);
				connectedComponent.addEdge(currentNodeID, neighbourNode);
			}
		}		
	}
	return connectedComponent;
}

int Algorithm::getConnectedComponentWithDFS(Graph &graph) {
	const int cAllNodesVisited = -1;
	int counterConnectedComponent = 0;
	int sizeGraph = graph.sizeNodes();
	int unvisitedNodeID = 0;
	std::vector<bool> nodesVisited(sizeGraph, false);

	while (cAllNodesVisited < unvisitedNodeID) { //besser: for i = letzter unbesuchter Knooten)
		depthFirstSearch(graph, unvisitedNodeID, nodesVisited);
		unvisitedNodeID = findUnvisitedNodeID(nodesVisited);
		counterConnectedComponent += 1;
	}
	return counterConnectedComponent;
}

std::vector<int> order;

void Algorithm::depthFirstSearch(Graph &graph, int nodeID, std::vector<bool> &visitedNodes) {
	int currentNodeID = nodeID;
	int neighbourNode = 0;
	visitedNodes[nodeID] = true;
	std::vector<int> neighbourIDsOfcurrentNode = graph.getNeighboursID(nodeID);

	for (int i = 0; i < neighbourIDsOfcurrentNode.size(); i++) {
		neighbourNode = neighbourIDsOfcurrentNode[i];

		if (!visitedNodes[neighbourNode]) {
			depthFirstSearch(graph, neighbourNode, visitedNodes);
			order.push_back(neighbourNode);
		}
	}
}

class compareEdges {
public:
	double operator() (Edge& p1, Edge& p2) {
		return p1.getWeight() > p2.getWeight();
	}
};

void pushEdgesInPQ(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges, std::vector<Edge> &edges);
Edge poplowestCostEdge(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges);
std::vector<Edge> findCandidateEdges(Node &currentNode, Graph &graph, Graph &mst);

Graph Algorithm::getPrimMinimumSpanningTree(Graph &graph) {
	Graph mst = Graph();
	std::priority_queue<Edge, std::vector<Edge>, compareEdges> candidateEdges;
	std::vector<Edge> candidates;
	int sizeGraph = graph.sizeNodes();
	Edge lowestCostEdge;
	Node newNode = graph.getNode(0);
		
	mst.addNode(newNode.getID());
	candidates = graph.getNodeEdges(newNode.getID());
	pushEdgesInPQ(candidateEdges, candidates);

	while (mst.sizeNodes() < sizeGraph && !candidateEdges.empty()) {
		lowestCostEdge = poplowestCostEdge(candidateEdges);

		if (!mst.nodeExist(lowestCostEdge.getNodeIDV1()) || !mst.nodeExist(lowestCostEdge.getNodeIDV2())) {
			newNode = !mst.nodeExist(lowestCostEdge.getNodeIDV1()) ? Node(lowestCostEdge.getNodeIDV1()) : Node(lowestCostEdge.getNodeIDV2());
			mst.addNode(newNode);
			mst.addEdge(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2(), lowestCostEdge.getWeight());
			mst.updateNeighbour(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2());
			candidates = findCandidateEdges(newNode, graph, mst);
			pushEdgesInPQ(candidateEdges, candidates);
		}
	}
	return mst;
}

void pushEdgesInPQ(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges, std::vector<Edge> &edges) {
	for (int i = 0; i < edges.size(); i++) {
		candidateEdges.push(edges[i]);
	}
}

Edge poplowestCostEdge(std::priority_queue<Edge, std::vector<Edge>, compareEdges> &candidateEdges) {
	Edge lowestCostEdge = candidateEdges.top();
	candidateEdges.pop();
	return lowestCostEdge;
}

std::vector<Edge> findCandidateEdges(Node &node, Graph &graph, Graph &mst) {
	std::vector<Edge> allEdges = graph.getNodeEdges(node.getID());
	std::vector<Edge> candidate;
	Edge edge;

	for (int i = 0; i < allEdges.size(); i++) {
		edge = allEdges[i];
		if (!mst.nodeExist(edge.getNodeIDV1()) || !mst.nodeExist(edge.getNodeIDV2())) {
			candidate.push_back(edge);
		}
	}
	return candidate;
}

bool your_comparer(Edge left, Edge right) {
	return left.getWeight() < right.getWeight();
}

bool isSelectedEdgeCreatingCycle(int set_u, int set_v);

double Algorithm::getKruskalMinimumSpanningTreeWeight(Graph &graph) {
	double mstCosts = 0;
	DisjointSets disjointSets(graph.sizeNodes());
	std::vector<Edge> edges;
	Edge edge;
	int nodeIDV1 = 0;
	int nodeIDV2 = 0;
	int set_node1 = 0;
	int set_node2 = 0;

	graph.copyEdgesInVector(edges);
	std::sort(edges.begin(), edges.end(), your_comparer);
	
	for (int i = 0; i < edges.size(); i++) { //TODO besser: zähler mst.size() < graph.size() mit zähler
		edge = edges[i];
		nodeIDV1 = edge.getNodeIDV1();
		nodeIDV2 = edge.getNodeIDV2();

		set_node1 = disjointSets.findParentOf(nodeIDV1);
		set_node2 = disjointSets.findParentOf(nodeIDV2);

		if (!isSelectedEdgeCreatingCycle(set_node1, set_node2)) {
			mstCosts += edges[i].getWeight();
			disjointSets.mergeTreesLowerToHigher(set_node1, set_node2);
		}
	}
	return mstCosts;
}

bool isSelectedEdgeCreatingCycle(int set_nodeID1, int set_nodeID2) {
	return set_nodeID1 == set_nodeID2;
}

Graph Algorithm::getNearestNeighborHamiltonianPath(Graph &graph, int startNodeID) {
	Graph hamPath = Graph();
	std::priority_queue<Edge, std::vector<Edge>, compareEdges> neighborEdges;
	std::vector<Edge> neighbor;
	int sizeGraph = graph.sizeNodes();
	Edge lowestCostEdge;
	Edge closingCycleEdge;
	Node startNode = graph.getNode(startNodeID);
	Node newNode = startNode;

	hamPath.addNode(newNode.getID());
	neighbor = graph.getNodeEdges(newNode.getID());
	pushEdgesInPQ(neighborEdges, neighbor);

	while (hamPath.sizeNodes() < sizeGraph) {
		lowestCostEdge = poplowestCostEdge(neighborEdges);

		while (!neighborEdges.empty()) {
			neighborEdges.pop();
		}

		if (!hamPath.nodeExist(lowestCostEdge.getNodeIDV1()) || !hamPath.nodeExist(lowestCostEdge.getNodeIDV2())) {
			newNode = !hamPath.nodeExist(lowestCostEdge.getNodeIDV1()) ? Node(lowestCostEdge.getNodeIDV1()) 
																	   : Node(lowestCostEdge.getNodeIDV2());
			hamPath.addNode(newNode);
			hamPath.addEdge(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2(), lowestCostEdge.getWeight());
			hamPath.updateNeighbour(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2());
			neighbor = findCandidateEdges(newNode, graph, hamPath);
			pushEdgesInPQ(neighborEdges, neighbor);
		}
	}
	closingCycleEdge = graph.getEdge(startNodeID, newNode.getID());
	hamPath.addEdge(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2(), closingCycleEdge.getWeight());
	hamPath.updateNeighbour(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2());
	return hamPath;
}

Graph Algorithm::getDoubleTreeHamiltonianPath(Graph &graph, int startNodeID) {
	Graph hamPath = Graph();
	Graph mst;
	Node nodeFirst = Node(startNodeID);
	Node nodeSecond;
	Edge edge;
	Edge closingCycleEdge;
	std::vector<int> order1;
	
	mst = Algorithm().getPrimMinimumSpanningTree(graph);
	std::vector<bool> nodesVisitedDFS(mst.sizeNodes(), false);
	depthFirstSearch(mst, startNodeID, nodesVisitedDFS);
	order1 = order;

	hamPath.addNode(startNodeID);

	std::cout << std::endl << std::endl;
	int counter = 0;


	for (int i = order1.size()-1; 0 <= i; i--) {
		nodeSecond = Node(order1[i]);
		order1.pop_back();
		edge = graph.getEdge(nodeFirst.getID(), nodeSecond.getID());


		std::cout << "Kante " << counter
			<< ": von " << edge.getNodeIDV1()
			<< " nach " << edge.getNodeIDV2()
			<< " mit Gewicht " << edge.getWeight() << std::endl;
		counter++;

		hamPath.addNode(nodeSecond);
		hamPath.addEdge(edge.getNodeIDV1(), edge.getNodeIDV2(), edge.getWeight());
		hamPath.updateNeighbour(edge.getNodeIDV1(), edge.getNodeIDV2());

		nodeFirst = nodeSecond;
	}

	std::cout << std::endl << std::endl;

	closingCycleEdge = graph.getEdge(startNodeID, nodeFirst.getID());
	hamPath.addEdge(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2(), closingCycleEdge.getWeight());
	hamPath.updateNeighbour(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2());

	return hamPath;

}

// Only for testing
void  Algorithm::test(Graph &graph) {
	Algorithm().getConnectedComponentWithDFS(graph);
}



