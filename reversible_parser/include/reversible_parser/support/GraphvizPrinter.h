/*
 * GraphvizPrinter.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_GRAPHVIZPRINTER_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_GRAPHVIZPRINTER_H_

#include <iostream>
#include <fstream>
#include <string>

namespace edsl {
namespace abstract_syntax_tree {

class GraphvizPrinter {
public:
	GraphvizPrinter(std::string filepath_dot, std::string filepath_png);
	virtual ~GraphvizPrinter();

	void writeNode(std::string node, std::string argument = "");
	void writeConnection(std::string a, std::string b, std::string argument = "");

	static std::string label(std::string text);


private:
	void openFile();
	void closeFile();
	void compileFile();

	void writeHeader();
	void writeFooter();
	static std::string format(std::string);

private:
	std::string _filepath_dot;
	std::string _filepath_png;
	std::ofstream _outputFile;
};

} /* namespace abstract_syntax_tree */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_GRAPHVIZPRINTER_H_ */
