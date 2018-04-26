#pragma once
#include "Graph.h"
#include "Node.h"
#include "DataUtils.h"
#include "Algorithm.h"
#include <iostream>
#include <string>

const std::string fromPathOfFile1 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph1.txt";
const std::string fromPathOfFile2 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph2.txt";
const std::string fromPathOfFile3 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph3.txt";
const std::string fromPathOfFile4 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph4.txt";

int main() {
	Graph mygraph = DataUtils().importGraphFromAdjMatrix(fromPathOfFile1);
//	Graph mygraph = DataUtils().importGraphFromAdjList(fromPathOfFile3);

	std::vector<Graph> conComponentBFS = Algorithm().getConnectedComponentWithBFS(mygraph);
	std::cout << "Der Graph besteht laut BFS aus " << conComponentBFS.size() << " Spannbaeumen." << std::endl;

	int conComponentDFS = Algorithm().getConnectedComponentWithDFS(mygraph);
	std::cout << "Der Graph besteht laut DFS aus " << conComponentDFS << " Spannbaeumen." << std::endl;

	system("PAUSE");
	return 0;
}