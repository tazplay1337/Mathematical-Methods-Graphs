#pragma once
#include "Algorithm.h"
#include "DisjointSets.h"
#include <iostream>
#include <utility>
#include <queue>
#include <stack>
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

std::unordered_map<int, int> bfsParentNodeDictionary;

Graph Algorithm::breadthFirstSearch(Graph &graph, int startNodeID) {
	Graph connectedComponent;
	bfsParentNodeDictionary.clear();

	if (graph.isDirected()) {
		connectedComponent = Graph(true);
	}

	int currentNodeID = startNodeID;
	int neighbourNode = 0;
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

				if (graph.isDirected()) {
					// double cost = graph.getEdgeCost(currentNodeID, neighbourNode);
					double capacity = graph.getEdgeCapacity(currentNodeID, neighbourNode);
					connectedComponent.addEdge(currentNodeID, neighbourNode, 0.0, capacity);
				}
				else {
					connectedComponent.addEdge(currentNodeID, neighbourNode);
				}
				bfsParentNodeDictionary.insert({ neighbourNode, currentNodeID });
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
//			mst.updateNeighbour(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2());
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
//			hamPath.updateNeighbour(lowestCostEdge.getNodeIDV1(), lowestCostEdge.getNodeIDV2());
			neighbor = findCandidateEdges(newNode, graph, hamPath);
			pushEdgesInPQ(neighborEdges, neighbor);
		}
	}
	closingCycleEdge = graph.getEdge(startNodeID, newNode.getID());
	hamPath.addEdge(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2(), closingCycleEdge.getWeight());
//	hamPath.updateNeighbour(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2());

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
//		hamPath.updateNeighbour(edge.getNodeIDV1(), edge.getNodeIDV2());

		nodeFirst = nodeSecond;
	}
	closingCycleEdge = graph.getEdge(startNodeID, nodeFirst.getID());
	hamPath.addEdge(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2(), closingCycleEdge.getWeight());
//	hamPath.updateNeighbour(closingCycleEdge.getNodeIDV1(), closingCycleEdge.getNodeIDV2());

	// TODO Löschen
	std::cout << "Kante " << counter
		<< ": von " << closingCycleEdge.getNodeIDV1()
		<< " nach " << closingCycleEdge.getNodeIDV2()
		<< " mit Gewicht " << closingCycleEdge.getWeight() << std::endl;
	std::cout << std::endl << std::endl;
	return hamPath;
}

void findOptimalTSP(struct originalGraphInfos &graphInfo, struct SequenceInfos &sequenceInfos, 
					std::vector<int> &nodeSequenceTSP, int currentNode, double currentCost, std::vector<bool> &nodeVisited);
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
	std::vector<bool> visitedNodes(graph.sizeNodes(), false);

	graphInfos.graph = graph;
	graphInfos.size = graph.sizeNodes();
	lowestCostSequenceInfos.useBranchBound = useBranchBound;
	
	findOptimalTSP(graphInfos, lowestCostSequenceInfos, nodeSequenceTSP, startNodeID, currentCost, visitedNodes);
	optimalTSP = bulidGraphFromSequence(graphInfos, lowestCostSequenceInfos);

	return optimalTSP;
}

void findOptimalTSP(originalGraphInfos &graphInfo, SequenceInfos &lowestCostSequenceInfos,
					std::vector<int> &currentSequence, int currentNode, double currentSequenceCost, std::vector<bool> &nodeVisited) {

//	currentSequence.push_back(currentNode);

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
//		currentSequence.pop_back();
	}
	else if(currentSequence.size() < graphInfo.size) { //unötig
		std::vector<int> neighbours = graphInfo.graph.getNeighboursID(currentNode);
		int neighbourID = -1;

		currentSequence.push_back(currentNode);
		

		for(int i = 0; i < neighbours.size(); i++) {
			neighbourID = neighbours[i];
		//	if (!nodeExist(currentSequence, neighbourID)) {
			if (!nodeVisited[neighbourID]) {			
				Edge edge = graphInfo.graph.getEdge(currentNode, neighbourID);
				double newCurrentCost = currentSequenceCost + edge.getWeight();
				// hier einfügen currentSequence
				// bool vektor
				nodeVisited[neighbourID] = true;
				findOptimalTSP(graphInfo, lowestCostSequenceInfos, currentSequence, neighbourID, newCurrentCost, nodeVisited);
				currentSequence.pop_back();
				nodeVisited[neighbourID] = false;
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
//		buildGraph.updateNeighbour(edge.getNodeIDV1(), edge.getNodeIDV2());
	}
	return buildGraph;
}

struct dijkstraNode {
	int id;
	int parentID = -1;
	double cost = std::numeric_limits<double>::max();
};

class compareDijkstraNode {
public:
	double operator() (dijkstraNode& n1, dijkstraNode& n2) {
		return n1.cost > n2.cost;
	}
};

Graph bulidGraphFromDijkstraNodes(Graph &orignalGraph, std::vector<dijkstraNode> &dijkstraNodes);
void printDijkstraNodes(std::vector<dijkstraNode> &dijkstraNodes, std::string algName);

Graph Algorithm::getShortestPathDijkstra(Graph &graph, int startNodeID) {

	if (graph.hasNegativeCostEdge()) {
		std::cout << "=== Dijkstra ===" << std::endl;
		std::cout << "Abbruch: negative Kanten Gewichte vorhanden." << std::endl << std::endl;
		return Graph();
	}

	Graph shortestPath;
	std::priority_queue<dijkstraNode, std::vector<dijkstraNode>, compareDijkstraNode> unprocessedNodesPQ;
	std::vector<bool> visitedNodes(graph.sizeNodes(), false);
	std::vector<int> neighbourIDsOfcurrentNode;
	std::vector<dijkstraNode> dijkstraNodes;
	dijkstraNode dijNode;
	int neighbourNodeID = 0;
	double costToNeighbor = 0;

	for (int i = 0; i < graph.sizeNodes(); i++) {
		dijNode.id = i;
		dijkstraNodes.push_back(dijNode);
	}

	dijkstraNodes[startNodeID].cost = 0;
	dijkstraNodes[startNodeID].parentID = startNodeID;

	unprocessedNodesPQ.push(dijkstraNodes[startNodeID]);
	
	while (!unprocessedNodesPQ.empty()) {
		dijNode = unprocessedNodesPQ.top();
		unprocessedNodesPQ.pop();
		
		if (!visitedNodes[dijNode.id]) {
			visitedNodes[dijNode.id] = true;
			neighbourIDsOfcurrentNode = graph.getNeighboursID(dijNode.id);

			for (size_t i = 0; i < neighbourIDsOfcurrentNode.size(); i++) {
				neighbourNodeID = neighbourIDsOfcurrentNode[i];
				costToNeighbor = dijNode.cost + graph.getEdgeCost(dijNode.id, neighbourNodeID);

				if (costToNeighbor < dijkstraNodes[neighbourNodeID].cost) {
					dijkstraNodes[neighbourNodeID].cost = costToNeighbor;
					dijkstraNodes[neighbourNodeID].parentID = dijNode.id;
					unprocessedNodesPQ.push(dijkstraNodes[neighbourNodeID]);
				}
			}
		}
	}
	shortestPath = bulidGraphFromDijkstraNodes(graph, dijkstraNodes);
	printDijkstraNodes(dijkstraNodes, "Dijkstra");
	return shortestPath;
}

Graph bulidGraphFromDijkstraNodes(Graph &orignalGraph, std::vector<dijkstraNode> &dijkstraNodes) {
	Graph shortestPath = Graph(true);
	Node node;
	Edge edge;
	dijkstraNode dijNode;
	int nodeID1 = 0;
	int nodeID2 = 0;
	double cost = 0;


	for (size_t i = 0; i < dijkstraNodes.size(); i++) {
		node = Node(i);
		shortestPath.addNode(node);
	}

	for (size_t i = 0; i < dijkstraNodes.size(); i++) {
		nodeID1 = dijkstraNodes[i].parentID;
		nodeID2 = dijkstraNodes[i].id;
		cost = orignalGraph.getEdgeCost(nodeID1, nodeID2);
		edge = Edge();
		shortestPath.addEdge(nodeID1, nodeID2, cost);
	}
	return shortestPath;
}

void printDijkstraNodes(std::vector<dijkstraNode> &dijkstraNodes, std::string algName) {
	std::cout << "=== " << algName << " ===" << std::endl;

	for (size_t i = 0; i < 50; i++) {
		std::cout << "Path to: " << dijkstraNodes[i].id
				  << " with cost: " << dijkstraNodes[i].cost << std::endl;
	}
	std::cout << std::endl << std::endl;
}

bool bellmanFordIteration(Graph &graph, std::vector<dijkstraNode> &dijkstraNodes);
void findNegativeCycle(Graph &graph, std::vector<dijkstraNode> &dijkstraNodes, int startNodeID);

Graph Algorithm::getShortestPathBellmanFord(Graph &graph, int startNodeID, bool &negativeCycleExist) {
	Graph shortestPath;
	std::vector<dijkstraNode> dijkstraNodes;
	dijkstraNode dijNode;

	for (int i = 0; i < graph.sizeNodes(); i++) {
		dijNode.id = i;
		dijkstraNodes.push_back(dijNode);
	}

	dijkstraNodes[startNodeID].cost = 0;

	dijkstraNodes[startNodeID].parentID = startNodeID;

	bool bellmanDetermined = true;
	bool costsImproved = true;
	bool hasNegativeCycle = false;

	for (int i = 0; costsImproved && i < graph.sizeNodes() - 1; i++) {
		costsImproved = bellmanFordIteration(graph, dijkstraNodes);

		if (!costsImproved) {
			bellmanDetermined = false;
		}
	}

	if (bellmanDetermined) {
		costsImproved = bellmanFordIteration(graph, dijkstraNodes);
		hasNegativeCycle = costsImproved;
	}

	if (hasNegativeCycle) {
		findNegativeCycle(graph, dijkstraNodes, startNodeID);
		negativeCycleExist = true;
		std::cout << "Der Graph enthaelt einen negativen Cyclus!" << std::endl;
	}
	else if(bellmanDetermined){
		shortestPath = bulidGraphFromDijkstraNodes(graph, dijkstraNodes);
	//	printDijkstraNodes(dijkstraNodes, "BellmanFord");
	}
	return shortestPath;
}

bool bellmanFordIteration(Graph &graph, std::vector<dijkstraNode> &dijkstraNodes) {
	dijkstraNode dijNode;
	std::vector<int> neighbourIDsOfcurrentNode;
	int neighbourNodeID = 0;
	double costToNeighbor = 0.0;
	bool costsImproved = false;

	for(int i=0; i < graph.sizeNodes(); i++){
		dijNode = dijkstraNodes[i];	
		neighbourIDsOfcurrentNode = graph.getNeighboursID(dijNode.id);

		for (size_t n = 0; n < neighbourIDsOfcurrentNode.size(); n++) {
			neighbourNodeID = neighbourIDsOfcurrentNode[n];
			costToNeighbor = dijNode.cost + graph.getEdgeCost(dijNode.id, neighbourNodeID);

			if (costToNeighbor < dijkstraNodes[neighbourNodeID].cost) {
				dijkstraNodes[neighbourNodeID].cost = costToNeighbor;
				dijkstraNodes[neighbourNodeID].parentID = dijNode.id;
				costsImproved = true;
			}
		}
	}
	return costsImproved;
}

int currendNodeAlreadyVisited(std::vector<int> negativeCycleEdges, int nodeID);

void findNegativeCycle(Graph &graph, std::vector<dijkstraNode> &dijkstraNodes, int startNodeID) {
	std::vector<dijkstraNode> cycle;
	std::vector<int> visitedNodes;
	std::vector<std::string> negativeCycleEdges;
	bool cycleFound = false;
	double cycleCost = 0.0;
	std::string edgeIndex;
	int startCycleNodeID = startNodeID;
	int indexNegCycleBeginn = 0;
//	dijkstraNode currendNode ;

	int currendNode = dijkstraNodes[startCycleNodeID].id;
	int parentNode = dijkstraNodes[startCycleNodeID].parentID;

	edgeIndex = std::to_string(parentNode) + ":" + std::to_string(currendNode);
	visitedNodes.push_back(currendNode);
	negativeCycleEdges.push_back(edgeIndex);

	while (true) {

		currendNode = dijkstraNodes[parentNode].id;
		parentNode = dijkstraNodes[parentNode].parentID;
		
		edgeIndex = std::to_string(parentNode) + ":" + std::to_string(currendNode);
		visitedNodes.push_back(currendNode);
		negativeCycleEdges.push_back(edgeIndex);

		indexNegCycleBeginn = currendNodeAlreadyVisited(visitedNodes, currendNode);

		if (indexNegCycleBeginn != -1) {
			visitedNodes.erase(visitedNodes.begin(), visitedNodes.begin() + 3);

		}
	}
}

int currendNodeAlreadyVisited(std::vector<int> negativeCycleEdges, int nodeID) {
	int alreadyVisited = -1;

	for (int i = 0; i < negativeCycleEdges.size(); i++) {	
		if (negativeCycleEdges[i] == nodeID) {
			alreadyVisited = i;
		}
	}
	return alreadyVisited;
}

Graph ResidualGraphGenerator(Graph &graph ,std::unordered_map<std::string, Edge> &edgesFlowGraph, 
							 std::unordered_map<std::string, bool> &indexForwardEdge, std::unordered_map<std::string, bool> &indexBackEdge);
Edge getCurrentToParentEdge(Graph &g, int sourceID);
std::string invertEdgeIndex(std::string edgeIndex);

Graph Algorithm::getMaxFlowEdmondsKarpAlgorithm(Graph &graph, int startNodeID, int targetNodeID) {
	Node hStartNode = graph.getNode(startNodeID);	

	Graph maxFlowGraph;
	std::unordered_map<std::string, Edge> edgesGraph = graph.getEdges();
	std::unordered_map<std::string, Edge> edgesFlowGraph;
	std::string edgeIndex;
	Edge edge_tmp;

	for (auto const& p : edgesGraph) {
		std::string edgeIndex = p.first;
		edge_tmp = p.second;
		edge_tmp.setWeight(0.0);
		edge_tmp.setCapacity(0.0);
		edgesFlowGraph.insert({ edgeIndex, edge_tmp });
	}
	
	std::unordered_map<std::string, bool> indexForwardEdge;
	std::unordered_map<std::string, bool> indexBackEdge;
	Graph residualGraph = ResidualGraphGenerator(graph, edgesFlowGraph, indexForwardEdge, indexBackEdge);

	Graph bfsGraph = breadthFirstSearch(residualGraph, startNodeID);

	double valTotalFlow = 0.0;

	while (bfsGraph.nodeExist(targetNodeID)) {
		double valAugmentationsValue = std::numeric_limits<double>::max();
		std::stack<std::string> bfsStackOfEdgeIndexPath;
		int currentNodeIdInBFSPath = targetNodeID;
		
		while (currentNodeIdInBFSPath != startNodeID) {
			Edge currentToParentEdge = getCurrentToParentEdge(bfsGraph, currentNodeIdInBFSPath);

			if (currentToParentEdge.getCapacity() < valAugmentationsValue) {
				valAugmentationsValue = currentToParentEdge.getCapacity();
			}
			edgeIndex = std::to_string(currentToParentEdge.getNodeIDV1()) + ":" + std::to_string(currentToParentEdge.getNodeIDV2());
			bfsStackOfEdgeIndexPath.push(edgeIndex);
			currentNodeIdInBFSPath = currentToParentEdge.getNodeIDV1();
		}

		valTotalFlow += valAugmentationsValue;

		std::string hBfsPathEdgeHash;

		std::cout << std::endl;

		while (!bfsStackOfEdgeIndexPath.empty()) {
			hBfsPathEdgeHash = bfsStackOfEdgeIndexPath.top();
			bfsStackOfEdgeIndexPath.pop();

			std::cout << "Edge: " << hBfsPathEdgeHash << std::endl;

			if (indexForwardEdge.find(hBfsPathEdgeHash) != indexForwardEdge.end()) {
				edgesFlowGraph[hBfsPathEdgeHash].addCapacity(valAugmentationsValue);
				edgesGraph[hBfsPathEdgeHash].addFlow(valAugmentationsValue);
			}
			else if (indexBackEdge.find(hBfsPathEdgeHash) != indexBackEdge.end()) {
				std::string indexOriginalEdge = invertEdgeIndex(hBfsPathEdgeHash);
				double subtractAugmentationsValue = valAugmentationsValue * -1;
				edgesFlowGraph[indexOriginalEdge].addCapacity(subtractAugmentationsValue);
				edgesGraph[indexOriginalEdge].addFlow(subtractAugmentationsValue);
			}
		}
		residualGraph = ResidualGraphGenerator(graph, edgesFlowGraph, indexForwardEdge, indexBackEdge);
		bfsGraph = breadthFirstSearch(residualGraph, startNodeID);
	}

	std::cout << std::endl;
	maxFlowGraph = Graph(true, graph.getNodes(), edgesGraph, valTotalFlow);
	return maxFlowGraph;
}

Graph ResidualGraphGenerator(Graph &graph, std::unordered_map<std::string, Edge> &edgesFlowGraph,
							 std::unordered_map<std::string, bool> &indexForwardEdge, std::unordered_map<std::string, bool> &indexBackEdge) {
	Graph residualGraph = Graph(true);
	indexBackEdge.clear();
	indexForwardEdge.clear();
	Node newNode;

	for (size_t id = 0; id < graph.sizeNodes(); id++) {
		newNode = Node(id);
		residualGraph.addNode(newNode);
	}

	std::unordered_map<std::string, Edge> edgesGraph = graph.getEdges();
	Edge edgeGraph;
	Edge edgeFlowGraph;
	int startNodeID = 0;
	int targetNodeID = 0;
	double residualCapacity = 0.0;
	double currentFlow = 0.0;
	std::string indexEdge;

	for (auto const& edgeRecord : edgesGraph) {
		indexEdge = edgeRecord.first;
		edgeGraph = edgeRecord.second;
		edgeFlowGraph = edgesFlowGraph[indexEdge];

		residualCapacity = edgeGraph.getCapacity() - edgeFlowGraph.getCapacity();
		currentFlow = edgeFlowGraph.getCapacity();

		startNodeID = edgeGraph.getNodeIDV1();
		targetNodeID = edgeGraph.getNodeIDV2();

         if (residualCapacity > 0.0) { // Hinkante einfügen?
			 residualGraph.addEdge(startNodeID, targetNodeID, 0.0, residualCapacity);
			 indexForwardEdge.insert({ indexEdge, true });
		 }

		if (currentFlow > 0.0) { // Rückkante einfügen?
			residualGraph.addEdge(targetNodeID, startNodeID, 0.0, currentFlow);
			indexEdge = std::to_string(targetNodeID) + ":" + std::to_string(startNodeID);
			indexBackEdge.insert({ indexEdge, true });
		}
	}
	return residualGraph;
}

Edge getCurrentToParentEdge(Graph &g, int sourceID) {
	int parentNode = bfsParentNodeDictionary[sourceID];
	return g.getEdge(parentNode, sourceID);;
}

std::string invertEdgeIndex(std::string edgeIndex) {
	int startIndexFirstNode = 0;
	int indexSeparator = edgeIndex.find(":");
	std::string firstNodeID = edgeIndex.substr(startIndexFirstNode, indexSeparator);
	std::string secondNodeID = edgeIndex.substr(indexSeparator + 1);
	std::string invertEdgeIndex = secondNodeID + ":" + firstNodeID;
	return invertEdgeIndex;
}

Graph residualCostCapacityGraphGenerator(Graph &graph, std::unordered_map<std::string, Edge> &edgesFlowGraph,
	std::unordered_map<std::string, bool> &indexForwardEdge, std::unordered_map<std::string, bool> &indexBackEdge);

std::vector<std::string> findNegativeCycle(Graph &graph, int startNodeID);

Graph Algorithm::getMinFlowCycleCancelingAlg(Graph &graph) {
	Graph minFlowGraph = graph;
	int freeNodeId = graph.sizeNodes();

	Node superStart = Node(freeNodeId, 0.0);
	minFlowGraph.addNode(superStart);

	Node superEnd = Node(freeNodeId + 1, 0.0);
	minFlowGraph.addNode(superEnd);

	std::unordered_map<int, Node>  minFlowNodes = minFlowGraph.getNodes();
	Node node;
	Edge newEdge;
	double balance = 0;
	double capacity = 0;

	for (auto const& nodeRecord : minFlowNodes) {
		node = nodeRecord.second;
		balance = node.getBalance();

		if (balance > 0.0) {
			capacity = node.getBalance();
			minFlowGraph.addEdge(superStart.getID(), node.getID(), 0.0, capacity);
		}
		else if (balance < 0.0) {
			capacity = node.getBalance() * -1.0;
			minFlowGraph.addEdge(node.getID(), superEnd.getID(), 0.0, capacity);
		}
	}

	// Nun soll der Edmonds-Karp Algorithmus ausgeführt werden. Dabei werden die SuperQuelle und SuperSenke verwendet
	Graph maxFlowGraph = getMaxFlowEdmondsKarpAlgorithm(minFlowGraph, superStart.getID(), superEnd.getID());
	superStart = minFlowGraph.getNode(superStart.getID());
	std::vector<int> superStartNeigbourIDs = superStart.getNeigbourIDs();
	Edge edgeSuperStartASource;
	double flowValue = 0.0;
	capacity = 0.0;

	// Nutzt die SuperQuelle die Kantenkapazitäten voll aus? 
	for (int i = 0; i < superStartNeigbourIDs.size(); i++) {
		edgeSuperStartASource = maxFlowGraph.getEdge(superStart.getID(), superStartNeigbourIDs[i]);
		flowValue = edgeSuperStartASource.getFlow();
		capacity = minFlowGraph.getEdgeCapacity(superStart.getID(), superStartNeigbourIDs[i]);

		if (!(flowValue == capacity)) {
			std::cout << "Error: No valid b-flow!" << std::endl;
			return Graph();
		}
	}

	maxFlowGraph.deleteNode(superStart.getID());
	maxFlowGraph.deleteNode(superEnd.getID());

	Graph residualGraph;
	std::unordered_map<int, Node>  residualNodes;
	std::unordered_map<std::string, bool> indexForwardEdge;
	std::unordered_map<std::string, bool> indexBackEdge;

	Graph shortestPathBFordGraph;
	bool negativeCycleExist = true;

	std::unordered_map<std::string, Edge> edgesMaxFlowGraph = maxFlowGraph.getEdges();

	while (negativeCycleExist) {

		residualGraph = residualCostCapacityGraphGenerator(graph, edgesMaxFlowGraph, indexForwardEdge, indexBackEdge);
		residualNodes = residualGraph.getNodes();

		// Ausgehend von jedem Knoten wird versucht ein negativer Zykel zu finden
		for (auto const& nodeRecord : residualNodes) {
			

			shortestPathBFordGraph = getShortestPathBellmanFord(residualGraph, nodeRecord.first, negativeCycleExist);

			if (negativeCycleExist) {

			}




		}


	}




	return Graph();
}

Graph residualCostCapacityGraphGenerator(Graph &graph, std::unordered_map<std::string, Edge> &edgesFlowGraph,
	std::unordered_map<std::string, bool> &indexForwardEdge, std::unordered_map<std::string, bool> &indexBackEdge) {
	
	Graph residualGraph = Graph(true);
	indexBackEdge.clear();
	indexForwardEdge.clear();
	Node newNode;

	for (size_t id = 0; id < graph.sizeNodes(); id++) {
		newNode = Node(id);
		residualGraph.addNode(newNode);
	}

	std::unordered_map<std::string, Edge> edgesGraph = graph.getEdges();
	Edge edgeGraph;
	Edge edgeFlowGraph;
	int startNodeID = 0;
	int targetNodeID = 0;
	double residualCapacity = 0.0;
	double currentFlow = 0.0;
	double cost = 0.0;
	std::string indexEdge;

	for (auto const& edgeRecord : edgesGraph) {
		indexEdge = edgeRecord.first;
		edgeGraph = edgeRecord.second;
		edgeFlowGraph = edgesFlowGraph[indexEdge];

		residualCapacity = edgeGraph.getCapacity() - edgeFlowGraph.getFlow();
		currentFlow = edgeFlowGraph.getFlow();
		cost = edgeFlowGraph.getWeight();

		startNodeID = edgeGraph.getNodeIDV1();
		targetNodeID = edgeGraph.getNodeIDV2();

		if (residualCapacity > 0.0) { // Hinkante einfügen?
			residualGraph.addEdge(startNodeID, targetNodeID, cost, residualCapacity);
			indexForwardEdge.insert({ indexEdge, true });
		}

		if (currentFlow > 0.0) { // Rückkante einfügen?
			cost = cost * -1;
			residualGraph.addEdge(targetNodeID, startNodeID, cost, currentFlow);
			indexEdge = std::to_string(targetNodeID) + ":" + std::to_string(startNodeID);
			indexBackEdge.insert({ indexEdge, true });
		}
	}
	return residualGraph;
}

/*
std::vector<std::string> findNegativeCycle(Graph &graph, int startNodeID) {
	
	std::vector<std::string> negativeCycle;

}*/


void  Algorithm::test(Graph &graph) {
	std::string edgeIndex = "1127:55454";
	int startIndexFirstNode = 0;
	int indexSeparator = edgeIndex.find(":");
	std::string firstNodeID = edgeIndex.substr(startIndexFirstNode, indexSeparator);
	std::string secondNodeID = edgeIndex.substr(indexSeparator + 1);
}




