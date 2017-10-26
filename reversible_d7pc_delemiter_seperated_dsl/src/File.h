/*
 * File.h
 *
 *  Created on: Apr 4, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_FILE_H_
#define REVERSIBLE_ASSEMBLY_SRC_FILE_H_

#include <iostream>
#include <fstream>
#include <vector>

class File
{

public:
	File(std::string filePath, std::string fileName);
	virtual ~File();

	std::vector<std::string> readLines();
	std::string readLine();
	bool isEndOfFile();


private:
	std::ifstream _inputFile;
};

#endif /* REVERSIBLE_ASSEMBLY_SRC_FILE_H_ */
