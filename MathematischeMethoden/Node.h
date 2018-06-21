#pragma once
#include <vector>

class Node{
public:
	Node();
	Node(int id);
	Node(int id, double balance);
	int getID();
	std::vector<int> getNeigbourIDs();
	void pushNeigbourID(int neigbourID);
	double getBalance();
	void setBalance(double balance);
	void removeNeigbour(double nodeID);
private:
	int id;
	double balance;
	std::vector<int> neigbourIDs;
};
