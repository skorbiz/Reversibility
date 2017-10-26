/*
 * GraphPrint.hpp
 *
 *  Created on: Feb 10, 2015
 *      Author: josl
 */

#ifndef GRAPHPRINT_HPP_
#define GRAPHPRINT_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <string>

#include <../src/language/AssemblyProgram.hpp>

#include <../src/model/basemodel/BaseProgram.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/expandedmodel/Sequence.hpp>
#include <../src/common/common.hpp>
#include <../src/common/GraphRepresentation.hpp>

namespace dsl {
namespace common {

class GraphPrint
{

public:
	GraphPrint(dsl::BaseProgram aAssemblyProgram);
	virtual ~GraphPrint();

private:
	void print();

	void openFile();
	void closeFile();

	std::string addQutationMark(std::string str);
	std::string generateColorsManual(std::string str);
	std::string generateColorsAutomatic(std::string str);
	std::string generateNodeProperties(std::shared_ptr<dsl::Instruction> a);
	std::string generateNodeArgument(std::shared_ptr<dsl::Instruction> a);

	void writeHeader();
	void writeFooter();
	void writeNode(std::shared_ptr<dsl::Instruction> a);
	void writeNode(std::string node, std::string argument = "");
	void writeConnectionDouble(std::shared_ptr<dsl::Instruction> a, std::shared_ptr<dsl::Instruction> b);
	void writeConnectionForward(std::shared_ptr<dsl::Instruction> a, std::shared_ptr<dsl::Instruction> b);
	void writeConnectionBackwards(std::shared_ptr<dsl::Instruction> a, std::shared_ptr<dsl::Instruction> b);
	void writeConnectionArgumented(std::shared_ptr<dsl::Instruction> a, std::shared_ptr<dsl::Instruction> b, std::string argument = "");
	void writeConnectionArgumented(std::string a, std::string b, std::string argument = "");

private:
	GraphRepresentation _graph;
	std::ofstream _outputFile;

};

} /* namespace common */
} /* namespace dsl */

#endif /* GRAPHPRINT_HPP_ */



