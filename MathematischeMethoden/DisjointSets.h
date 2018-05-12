#pragma once
class DisjointSets {
private:
	int *parent;
	int *rank;
	int size;

public:
	DisjointSets();
	DisjointSets(int size);
	int findParentOf(int node);
	void mergeTreesLowerToHigher(int x, int y);
};