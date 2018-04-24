#pragma once
#include <iostream>
#include <string>
using namespace std;




int main() {


	std::string hash = std::to_string(42) + ":" + std::to_string(452);

	cout << hash << endl;

	system("PAUSE");
	return 0;
}