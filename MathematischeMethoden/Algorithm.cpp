#pragma once
#include "Algorithm.h"
#include "DisjointSets.h"
#include <iostream>
#include <utility>
#include <queue>
#include <limits>

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

std::vector<int> cOrder;

void Algorithm::depthFirstSearch(Graph &graph, int nodeID, std::vector<bool> &visitedNodes) {
	int currentNodeID = nodeID;
	int neighbourNode = 0;
	visitedNodes[nodeID] = true;
	std::vector<int> neighbourIDsOfcurrentNode = graph.getNeighboursID(nodeID);

	for (int i = 0; i < neighbourIDsOfcurrentNode.size(); i++) {
		neighbourNode = neighbourIDsOfcurrentNode[i];

		if (!visitedNodes[neighbourNode]) {
			depthFirstSearch(graph, neighbourNode, visitedNodes);
			cOrder.push_back(neighbourNode);
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

	// TODO Löschen
	int counter = 0;
	std::cout << "getDoubleTreeHamiltonianPath" << std::endl;

	while (hamPath.sizeNodes() < sizeGraph) {
		lowestCostEdge = poplowestCostEdge(neighborEdges);

		// TODO Löschen
		std::cout << "Kante " << counter
			<< ": von " << lowestCostEdge.getNodeIDV1()
			<< " nach " << lowestCostEdge.getNodeIDV2()
			<< " mit Gewicht " << lowestCostEdge.getWeight() << std::endl;
		counter += 1;

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

	// TODO Löschen
	std::cout << "Kante " << counter
		<< ": von " << closingCycleEdge.getNodeIDV1()
		<< " nach " << closingCycleEdge.getNodeIDV2()
		<< " mit Gewicht " << closingCycleEdge.getWeight() << std::endl;
	std::cout << std::endl << std::endl;
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
	order1 = cOrder;

	hamPath.addNode(startNodeID);

	// TODO Löschen
	int counter = 0;
	std::cout << "getDoubleTreeHamiltonianPath" << std::endl;

	for (int i = order1.size()-1; 0 <= i; i--) {
		nodeSecond = Node(order1[i]);
	//	order1.pop_back(); unnötig
		edge = graph.getEdge(nodeFirst.getID(), nodeSecond.getID());

		// TODO Löschen
		std::cout << "Kante " << counter
			<< ": von " << edge.getNodeIDV1()
			<< " nach " << edge.getNodeIDV2()
			<< " mit Gewicht " << edge.getWeight() << std::endl;
		counter += 1;

		hamPath.addNode(nodeSecond);
		hamPath.addEdge(edge.getNodeIDV1(), edge.getNodeIDV2(), edge.getWeight());
		hamPath.updateNeighbour(edge.getNodeIDV1(), edge.getNodeIDV2());

		nodeFirst = nodeSecond;
	}
	closingCycleEdge = graph.getEdge(startNodeID, nodeFirst.getID());
	hamPath.addEdge(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2(), closingCycleEdge.getWeight());
	hamPath.updateNeighbour(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2());

	// TODO Löschen
	std::cout << "Kante " << counter
		<< ": von " << closingCycleEdge.getNodeIDV1()
		<< " nach " << closingCycleEdge.getNodeIDV2()
		<< " mit Gewicht " << closingCycleEdge.getWeight() << std::endl;
	std::cout << std::endl << std::endl;
	return hamPath;
}

void findOptimalTSP(struct originalGraphInfos &graphInfo, struct SequenceInfos &sequenceInfos, 
					std::vector<int> nodeSequenceTSP, int currentNode, double currentCost);
bool nodeExist(std::vector<int> &nodeSequenceTSP, int nodeID);
Graph bulidGraphFromSequence(struct originalGraphInfos &graphInfo, struct SequenceInfos &sequenceInfos);

struct originalGraphInfos {
	Graph graph;
	int size = 0;
};

struct SequenceInfos {
	std::vector<int> sequence;
	double cost = std::numeric_limits<double>::max();
	bool useBranchBound = true;
};

Graph Algorithm::getOptimalTSP(Graph &graph, bool useBranchBound){
	Graph optimalTSP;
	originalGraphInfos graphInfos;
	SequenceInfos lowestCostSequenceInfos;
	std::vector<int> nodeSequenceTSP;
	int startNodeID = 0;
	double currentCost = 0;

	graphInfos.graph = graph;
	graphInfos.size = graph.sizeNodes();
	lowestCostSequenceInfos.useBranchBound = useBranchBound;
	
	findOptimalTSP(graphInfos, lowestCostSequenceInfos, nodeSequenceTSP, startNodeID, currentCost);
	optimalTSP = bulidGraphFromSequence(graphInfos, lowestCostSequenceInfos);

	return optimalTSP;
}

void findOptimalTSP(originalGraphInfos &graphInfo, SequenceInfos &lowestCostSequenceInfos,
					std::vector<int> currentSequence, int currentNode, double currentSequenceCost) {

	currentSequence.push_back(currentNode);

	if (lowestCostSequenceInfos.useBranchBound && currentSequenceCost > lowestCostSequenceInfos.cost) {
		return;
	}

	if (currentSequence.size() == graphInfo.size) {
		Edge closingCycleEdge = graphInfo.graph.getEdge(0, currentNode);
		currentSequence.push_back(0);
		currentSequenceCost = currentSequenceCost + closingCycleEdge.getWeight();

		if (currentSequenceCost < lowestCostSequenceInfos.cost){
			lowestCostSequenceInfos.cost = currentSequenceCost;
			lowestCostSequenceInfos.sequence = currentSequence;
		}
	}
	else if(currentSequence.size() < graphInfo.size) { //unötig
		std::vector<int> neighbours = graphInfo.graph.getNeighboursID(currentNode);
		int neighbourID = -1;

		for(int i = 0; i < neighbours.size(); i++) {
			neighbourID = neighbours[i];

			if (!nodeExist(currentSequence, neighbourID)) {
				Edge edge = graphInfo.graph.getEdge(currentNode, neighbourID);
				double newCurrentCost = currentSequenceCost + edge.getWeight();
				// hier einfügen currentSequence
				// bool vektor
				findOptimalTSP(graphInfo, lowestCostSequenceInfos, currentSequence, neighbourID, newCurrentCost);
				// bool vektor 
				// hier wegtun currentSequence
			}
		}
	}
}

bool nodeExist(std::vector<int> &nodeSequence, int nodeID) {

	for (int i = 0; i < nodeSequence.size(); i++) {
		if (nodeID == nodeSequence[i]) {
			return true;
		}
	}
	return false;
}

Graph bulidGraphFromSequence(struct originalGraphInfos &graphInfo, struct SequenceInfos &sequenceInfo) {
	Graph buildGraph = Graph();
	Edge edge;
	int nodeFirstID = 0;
	int nodeSecondID = 0;
	
	for (int i = 0; i < graphInfo.size; i++) {
		nodeFirstID = sequenceInfo.sequence[i];
		nodeSecondID = sequenceInfo.sequence[i+1];

		edge = graphInfo.graph.getEdge(nodeFirstID, nodeSecondID);
		buildGraph.addNode(nodeFirstID);
		buildGraph.addEdge(edge.getNodeIDV1(), edge.getNodeIDV2(), edge.getWeight());
		buildGraph.updateNeighbour(edge.getNodeIDV1(), edge.getNodeIDV2());
	}
	return buildGraph;
}

void  Algorithm::test(Graph &graph) {
}



