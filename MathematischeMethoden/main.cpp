#pragma once
#include "Graph.h"
#include "Node.h"
#include "DataUtils.h"
#include "Algorithm.h"
#include <iostream>
#include <string>
#include <time.h>

const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\Beispiel.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_1_2.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_1_20.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_1_200.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_10_20.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_10_200.txt";
//const std::string fromPathOfFile = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Praktikum 2\\G_100_200.txt";

int main() {
	/*
	clock_t clockStart;
	clock_t clockEnd;
	float diffClockStartEnd;
	float entireTerm;*/

	Graph mygraph = DataUtils().importGraphFromAdjList(fromPathOfFile);
	Algorithm().test(mygraph);

	/*
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
	clockStart = clock();
	std::cout << "Cost MST of Prim : " << mst.totalCost() << std::endl;
	clockEnd = clock();
	
	diffClockStartEnd = (float)clockEnd - (float)clockStart;
	entireTerm = diffClockStartEnd / CLOCKS_PER_SEC;
	std::cout << std::endl << "Time to Calc Totalcost: " << entireTerm << std::endl;
*/
	system("PAUSE");
	return 0;
}










/*Praktikum 1
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