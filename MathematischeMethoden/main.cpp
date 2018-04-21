#pragma once
#include "Graph.h"
#include "Node.h"
#include "DataUtils.h"
#include "Algorithm.h"
#include <iostream>
#include <string>
using namespace std;

const string fromPathOfFile1 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph1.txt";
const string fromPathOfFile2 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph2.txt";
const string fromPathOfFile3 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph3.txt";
const string fromPathOfFile4 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph4.txt";

int main(){
	Graph mygraph = DataUtils().importGraphFromAdjList(fromPathOfFile2);
	Algorithm().getConnectedComponentWithBFS(mygraph);

	system("PAUSE");
	return 0;
}