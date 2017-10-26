/*
 * GraphPrint.hpp
 *
 *  Created on: Feb 10, 2015
 *      Author: josl
 */

#ifndef GRAPHPRINT_HPP_
#define GRAPHPRINT_HPP_

#include <model/graphviz/GraphRepresentation.hpp>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>

namespace model {
class Instruction;
} /* namespace model */

namespace dsl {
namespace common {

class GraphPrint
{

public:
	GraphPrint(GraphRepresentation graph);
	virtual ~GraphPrint();

private:
	void print();

	void openFile();
	void closeFile();

	std::string addQutationMark(std::string str);
	std::string generateColorsManual(std::string str);
	std::string generateColorsAutomatic(std::string str);
	std::string generateNodeArgument(std::shared_ptr<model::Instruction> a);
	std::string generateNodeProperties(std::shared_ptr<model::Instruction> a);
	std::string generateNodeIdentification(std::shared_ptr<model::Instruction> a);

	void writeHeader();
	void writeFooter();
	void writeNode(std::shared_ptr<model::Instruction> a);
	void writeNode(std::string node, std::string argument = "");
	void writeConnectionDouble(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b);
	void writeConnectionForward(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b);
	void writeConnectionBackwards(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b);
	void writeConnectionArgumented(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b, std::string argument = "");
	void writeConnectionArgumented(std::string a, std::string b, std::string argument = "");

private:
	GraphRepresentation _graph;
	std::ofstream _outputFile;

};

} /* namespace common */
} /* namespace dsl */

#endif /* GRAPHPRINT_HPP_ */



