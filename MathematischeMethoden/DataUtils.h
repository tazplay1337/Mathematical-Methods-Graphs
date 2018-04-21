#pragma once
#include "Graph.h"
#include <string>

class DataUtils {
public:
	DataUtils();
	static Graph importGraphFromAdjList(std::string fromPathOfFile);
	static Graph importGraphFromAdjMatrix(std::string fromPathOfFile);
};