/*
 * GraphvizPrinter.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <support/GraphvizPrinter.h>
#include <boost/regex.hpp>
#include <cstdlib>

namespace edsl {
namespace abstract_syntax_tree {


GraphvizPrinter::GraphvizPrinter(std::string filepath_dot, std::string filepath_png) :
		_filepath_dot(filepath_dot),
		_filepath_png(filepath_png)
{
	openFile();
	writeHeader();

}

GraphvizPrinter::~GraphvizPrinter()
{
	writeFooter();
	closeFile();
	compileFile();
}

void GraphvizPrinter::openFile()
{
	_outputFile.open(_filepath_dot);

	if (_outputFile.is_open() == false)
	{
		std::cerr << "Failed to open file in GraphPrint" << std::endl;
		exit(-1);
	}

}

void GraphvizPrinter::closeFile()
{
	_outputFile.close();
}

void GraphvizPrinter::compileFile()
{
	std::string input = _filepath_dot;
	std::string output = _filepath_png;
	std::string cmd = "dot -Tpng " + input + " -o " + output;
	system(cmd.c_str());
}

void GraphvizPrinter::writeHeader()
{
	_outputFile << "digraph graphname {";
}

void GraphvizPrinter::writeFooter()
{
	_outputFile << "\n }" << std::endl;
}

// **********************************
// *** LOGIC AND INPUTS

void GraphvizPrinter::writeNode(std::string node, std::string argument)
{
	_outputFile << "\n\"" << node << "\"[" << argument << "];";
}

void GraphvizPrinter::writeConnection(std::string a, std::string b, std::string argument)
{
	_outputFile << "\n";
	_outputFile << "\"";
	_outputFile << a;
	_outputFile << "\" -> \"";
	_outputFile << b;
	_outputFile << "\" ";
	_outputFile << argument;
	_outputFile << ";";

}

// **********************************
// *** support

std::string GraphvizPrinter::label(std::string text)
{
	return "label = \"" + format(text) + "\",";
}

std::string GraphvizPrinter::format(std::string text)
{
	  boost::regex expr{"\""};
	  std::string fmt{""};
	  return boost::regex_replace(text, expr, fmt);
//	  return text;
}


} /* namespace abstract_syntax_tree */
} /* namespace edsl */
