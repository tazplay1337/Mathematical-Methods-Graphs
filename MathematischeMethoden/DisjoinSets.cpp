#pragma once
#include "DisjointSets.h"

DisjointSets::DisjointSets() {
}

void initialNodesInDifferentSetsAndParentOfItself(int size, int *parent, int *rank);

DisjointSets::DisjointSets(int size) {
	this->size = size;
	this->parent = new int[size]; // adffgf ToDO
	this->rank = new int[size];
	initialNodesInDifferentSetsAndParentOfItself(size, parent, rank);
}

void initialNodesInDifferentSetsAndParentOfItself(int size, int *parent, int *rank) {
	for (int i = 0; i < size; i++){ //TODO
		rank[i] = 0;
		parent[i] = i;
	}
}

int DisjointSets::findParentOf(int node) {
	if (node != parent[node]){
		parent[node] = findParentOf(parent[node]);
	}
	return parent[node];
}

void DisjointSets::mergeTreesLowerToHigher(int nodeID1, int nodeID2) {
	nodeID1 = findParentOf(nodeID1);
	nodeID2 = findParentOf(nodeID2);
	if (rank[nodeID1] > rank[nodeID2]) {
		parent[nodeID2] = nodeID1;
	}
	else {
		parent[nodeID1] = nodeID2;
	}

	if (rank[nodeID1] == rank[nodeID2]) {
		rank[nodeID2] += 1;
	}
}