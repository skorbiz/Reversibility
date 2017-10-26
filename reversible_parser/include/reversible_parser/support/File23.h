/*
 * File.h
 *
 *  Created on: Apr 4, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_DEMO_SRC_FILE23_H_
#define REVERSIBLE_DEMO_SRC_FILE23_H_

#include <iostream>
#include <fstream>
#include <vector>

class File23
{

public:
//	File23();
	File23(std::string filePath, std::string fileName);
	virtual ~File23();


	std::vector<std::string> readLines();
	std::string readLine();
	std::string readFile();
	bool isEndOfFile();


private:
	std::ifstream _inputFile;
};

#endif /* REVERSIBLE_DEMO_SRC_FILE23_H_ */
