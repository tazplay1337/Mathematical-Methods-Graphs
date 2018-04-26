#pragma once
#include <vector>

class Node{
public:
	Node();
	Node(int id);
	int getID();
	std::vector<int> getNeigbourIDs(int neigbourID);
	void pushNeigbourID(int neigbourID);
private:
	int id;
	std::vector<int> neigbourIDs;
};
