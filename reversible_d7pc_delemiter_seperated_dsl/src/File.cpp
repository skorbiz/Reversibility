/*
 * File.cpp
 *
 *  Created on: Apr 4, 2017
 *      Author: josl
 */

#include "File.h"

File::File(std::string filePath, std::string fileName)
{
	_inputFile.open(filePath + fileName);

	if (_inputFile.is_open() == false)
	{
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "Failed to open file" << std::endl;
		std::cerr << "FileName: " << fileName << std::endl;
		std::cerr << "FilePath: " << filePath << std::endl;
		std::cerr << "File: " << filePath + fileName << std::endl;
		exit(-1);
	}
}

std::vector<std::string> File::readLines()
{
	std::vector<std::string> lines;
	while(_inputFile.peek() != EOF)
		lines.push_back(readLine());
	return lines;
}

std::string File::readLine()
{
	std::string line;
	std::getline(_inputFile,line);
	return line;
}

bool File::isEndOfFile()
{
	return _inputFile.peek() == EOF;
}

File::~File()
{
	_inputFile.close();
}

