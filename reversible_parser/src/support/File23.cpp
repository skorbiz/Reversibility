/*
 * File.cpp
 *
 *  Created on: Apr 4, 2017
 *      Author: josl
 */

#include <support/File23.h>

//File23::File23()
File23::File23(std::string filePath, std::string fileName)
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

std::vector<std::string> File23::readLines()
{
	std::vector<std::string> lines;
	while(_inputFile.peek() != EOF)
		lines.push_back(readLine());
	return lines;
}

std::string File23::readLine()
{
	std::string line;
	std::getline(_inputFile,line);
	return line;
}

std::string File23::readFile()
{
	std::string file;
	while(_inputFile.peek() != EOF)
		file.append(readLine() + "\n");
	return file;
}

bool File23::isEndOfFile()
{
	return _inputFile.peek() == EOF;
}

File23::~File23()
{
	_inputFile.close();
}

