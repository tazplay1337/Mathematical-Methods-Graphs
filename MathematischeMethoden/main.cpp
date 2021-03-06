#pragma once
#include "Graph.h"
#include "Node.h"
#include "DataUtils.h"
#include "Algorithm.h"
#include <iostream>
#include <string>
#include <time.h>

const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 6\\Kostenminimal0c.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 6\\Kostenminimal1.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 6\\Kostenminimal2.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 6\\Kostenminimal3.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 6\\Kostenminimal4.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 6\\Kostenminimal5.txt";

int main() {
	Graph mygraph = DataUtils().importDirectedBalanceGraphFromAdjList(fromPathOfFile);

	Graph mFlowGraph = Algorithm().getMinFlowCycleCancelingAlg(mygraph);

	std::cout << "Min Fluss: " << mFlowGraph.totalFlowCost() << std::endl;

	system("PAUSE");
	return 0;
}


/* Praktikum 1
const std::string fromPathOfFile1 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph1.txt";
const std::string fromPathOfFile2 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph2.txt";
const std::string fromPathOfFile3 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph3.txt";
const std::string fromPathOfFile4 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph4.txt";

Graph mygraph = DataUtils().importGraphFromAdjMatrix(fromPathOfFile1);
Graph mygraph = DataUtils().importGraphFromAdjList(fromPathOfFile4);

std::vector<Graph> conComponentBFS = Algorithm().getConnectedComponentWithBFS(mygraph);
std::cout << "Der Graph besteht laut BFS aus " << conComponentBFS.size() << " Spannbaeumen." << std::endl;

int conComponentDFS = Algorithm().getConnectedComponentWithDFS(mygraph);
std::cout << "Der Graph besteht laut DFS aus " << conComponentDFS << " Spannbaeumen." << std::endl;
*/


/* Praktikum 2
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\Beispiel.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_1_2.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_1_20.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_1_200.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_10_20.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_10_200.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_100_200.txt";

clock_t clockStart;
clock_t clockEnd;
float diffClockStartEnd;
float entireTerm;

clockStart = clock();
Graph mygraph = DataUtils().importGraphFromAdjList(fromPathOfFile);
clockEnd = clock();

diffClockStartEnd = (float)clockEnd - (float)clockStart;
entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;
std::cout << "Time to Import Graph: " << entireTerm << std::endl << std::endl;


clockStart = clock();
Graph mst = Algorithm().getPrimMinimumSpanningTree(mygraph);
clockEnd = clock();
diffClockStartEnd = (float)clockEnd - (float)clockStart;
entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;

std::cout << "Number Nodes of MST: " << mst.sizeNodes() << std::endl;
std::cout << "Number Edges of MST: " << mst.sizeEdges() << std::endl;
std::cout << "Entire Term of Prim: " << entireTerm << std::endl;
std::cout << "Cost MST of Prim : " << mst.totalCost() << std::endl << std::endl;

clockStart = clock();
double mstWeight = Algorithm().getKruskalMinimumSpanningTree(mygraph);
clockEnd = clock();
diffClockStartEnd = (float)clockEnd - (float)clockStart;
entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;

std::cout << "Entire Term of Kruskal: " << entireTerm << std::endl;
std::cout << "Cost MST of Kruskal : " << mstWeight << std::endl;
*/


/* Praktikum 3
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 3\\G_0.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 3\\K_10.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 3\\K_10e.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 3\\K_12.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 3\\K_12e.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 3\\K_15.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 3\\K_15e.txt";

clock_t clockStart;
clock_t clockEnd;
float diffClockStartEnd;
float entireTerm;
int startNode = 0;
Graph hamPath;

// Import
clockStart = clock();
Graph mygraph = DataUtils().importGraphFromAdjList(fromPathOfFile);
clockEnd = clock();

diffClockStartEnd = (float)clockEnd - (float)clockStart;
entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;
std::cout << "Time to Import Graph: " << entireTerm << std::endl << std::endl;

// DoubleTree
clockStart = clock();
hamPath = Algorithm().getDoubleTreeHamiltonianPath(mygraph, startNode);
clockEnd = clock();
diffClockStartEnd = (float)clockEnd - (float)clockStart;
entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;

std::cout << "DoubleTree: " << std::endl;
std::cout << "Startnode: " << startNode << std::endl;
std::cout << "Number Nodes: " << hamPath.sizeNodes() << std::endl;
std::cout << "Number Edges: " << hamPath.sizeEdges() << std::endl;
std::cout << "Entire Term: " << entireTerm << std::endl;
std::cout << "Cost : " << hamPath.totalCost() << std::endl << std::endl;

// findLowestCostTSP
clockStart = clock();
hamPath = Algorithm().getOptimalTSP(mygraph, false);
clockEnd = clock();
diffClockStartEnd = (float)clockEnd - (float)clockStart;
entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;

std::cout << "Test all Solutions: " << std::endl;
std::cout << "Number Nodes of TSP all Tours: " << hamPath.sizeNodes() << std::endl;
std::cout << "Number Edges: " << hamPath.sizeEdges() << std::endl;
std::cout << "Entire Term: " << entireTerm << std::endl;
std::cout << "Cost: " << hamPath.totalCost() << std::endl << std::endl;

// findLowestCostTSP
clockStart = clock();
hamPath = Algorithm().getOptimalTSP(mygraph);
clockEnd = clock();
diffClockStartEnd = (float)clockEnd - (float)clockStart;
entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;

std::cout << "Branch and Bound: " << std::endl;
std::cout << "Number Nodes of TSP all Tours: " << hamPath.sizeNodes() << std::endl;
std::cout << "Number Edges: " << hamPath.sizeEdges() << std::endl;
std::cout << "Entire Term: " << entireTerm << std::endl;
std::cout << "Cost: " << hamPath.totalCost() << std::endl << std::endl;

*/

/* Praktikum 4
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 4\\Wege1.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 4\\Wege2.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 4\\Wege3.txt";
const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 4\\G_1_2.txt";

int startnode = 0;
//Graph mygraph = DataUtils().importDirectedGraphFromAdjList(fromPathOfFile);
Graph mygraph = DataUtils().importGraphFromAdjList(fromPathOfFile);

std::cout << "Shortest Graph from Node: " << startnode << std::endl << std::endl;

Graph spg = Algorithm().getShortestPathDijkstra(mygraph, startnode);

spg = Algorithm().getShortestPathBellmanFord(mygraph, startnode);

*/

/*
const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 5\\Fluss.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 5\\Fluss2.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 5\\G_1_2.txt";

int startnode = 0;
int targetnode = 7;
Graph mygraph = DataUtils().importDirectedGraphFromAdjList(fromPathOfFile);

Graph spg = Algorithm().getMaxFlowEdmondsKarpAlgorithm(mygraph, startnode, targetnode);

std::cout << "Max Fluss: " << spg.getValTotalFlow() << std::endl;
*/