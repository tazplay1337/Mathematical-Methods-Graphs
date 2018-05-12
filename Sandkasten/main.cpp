#pragma once
#include <iostream>
#include <string>
#include <utility>
#include <queue>
using namespace std;




int main() {

	std::pair<int, int> edge1 = std::make_pair(1, 2);
	std::pair<int, int> edge2 = std::make_pair(1, 3);
	std::pair<int, int> edge3 = std::make_pair(3, 4);
	std::pair<int, int> edge4 = std::make_pair(4, 1);


	std::pair<double, std::pair<int, int>> wEdge1 = std::make_pair(-0.111, edge1);
	std::pair<double, std::pair<int, int>> wEdge2 = std::make_pair(-0.45, edge2);
	std::pair<double, std::pair<int, int>> wEdge3 = std::make_pair(-0.33335, edge3);
	std::pair<double, std::pair<int, int>> wEdge4 = std::make_pair(-0.5575, edge4);


	std::priority_queue<std::pair<double, std::pair<int, int>>> mypq;
	mypq.push(wEdge1);
	mypq.push(wEdge2);
	mypq.push(wEdge3);
	mypq.push(wEdge4);

	std::cout << "My Tree: " << std::endl;

	while (!mypq.empty()) {
		std::pair<double, std::pair<int, int>> wEdge = mypq.top();;
		std::cout << "Weight is : " << wEdge.first << " from Node: " << wEdge.second.first << " to Node: " << wEdge.second.second << std::endl;
		mypq.pop();
	}

	system("PAUSE");
	return 0;
}