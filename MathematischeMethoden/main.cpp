#pragma once
#include "Graph.h"
#include "Node.h"
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

const string pathOfGraphtxt1 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph1.txt";
const string pathOfGraphtxt2 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph2.txt";
const string pathOfGraphtxt3 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph3.txt";
const string pathOfGraphtxt4 = "C:\\Users\\taz\\Dropbox\\00 Rund ums Studium\\Mathematische Methoden\\Graph4.txt";

Graph getGraphFromInputFile(string pathOfFile);

int main(){

	// Erstelle Graphen

	// Lese Matrix ein

	// Finge herraus wie viele einzelkomponeten es gibt


	string dataInput = "";
	ifstream file;
	
	ifstream input(pathOfGraphtxt2);

	if (input) {       // Überprüfung ob Datei geoeffnet wurde oder existiert
		string line;
		while(getline(input, line))
		cout << line << endl;

	}


/*
	vector<int> nodeList; // Neue Knotenliste erstellt
	string dataInput = "";
	file >> dataInput;
	int temp = 0;

	for (int i = 0; i < dataInput.size(); i++) // Nur Ganzzahlen zwischen 0 und 9 werden eingelesen
	{
		if (dataInput[i] != '{' && dataInput[i] != '}' && dataInput[i] != ',')
		{
			if (dataInput[i] >= '0' && dataInput[i] <= '9')
			{
				temp = dataInput[i] - '0';
				nodeList.push_back(temp);
			}
		}
	}
	graph.setNodelist(nodeList); // Knotenliste wird initialisiert
	map<int, vector<int> > adjList = graph.getAdjlist(); // Adjazenzliste zur Knotenliste wird erstellt
*/
	system("PAUSE");
	return 0;
}

Graph getGraphFromInputFile(string pathOfFile) {

	return Graph();
}