/*
 * GraphPrint.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: josl
 */

#include <model/graphviz/GraphPrint.hpp>
#include <model/Identification23.h>
#include <model/Instruction.h>
#include <ros/package.h>
#include <cstdlib>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <vector>

namespace dsl {
namespace common {


GraphPrint::GraphPrint(GraphRepresentation graph) :
		_graph(graph)
{
	openFile();
	writeHeader();
	print();
	writeFooter();
	closeFile();
}

GraphPrint::~GraphPrint()
{
}


void GraphPrint::print()
{
	int maxRank = _graph.getMaxRank();

	if(maxRank > 10000)
	{
		std::cerr << "GraphPrint: max rank as: " << maxRank << std::endl;
		exit(-1);
	}

	for(int r = 0; r < maxRank; r++)
	{
		std::vector<std::shared_ptr<model::Instruction> > ins = _graph.getInstructionOfRank(r);
			_outputFile << "\n {rank = same; \"r" << r << "\"";
			for(unsigned int i = 0; i < ins.size(); i++)
				_outputFile << " \"" << generateNodeIdentification(ins[i]) << "\"";
			_outputFile << "};";
	}

	for(int r = 0; r <= maxRank; r++)
		_outputFile << "\n \"r" << std::to_string(r) << "\"" << "[style=invis];";

	for(int r = 0; r < maxRank; r++)
		writeConnectionArgumented("r"+std::to_string(r), "r"+std::to_string(r+1), "[style=invis]");

	std::vector<std::shared_ptr<model::Instruction> > instructions = _graph.getInstructions();
	for(unsigned int i = 0; i < instructions.size(); i++)
		writeNode(instructions[i]);

	std::vector<std::shared_ptr<model::Instruction> > edgeDouble = _graph.getOmniDirectionalEdges();
	for(unsigned int e = 0; e < edgeDouble.size(); e=e+2)
		writeConnectionDouble(edgeDouble[e],edgeDouble[e+1]);

	std::vector<std::shared_ptr<model::Instruction> > edgeForward = _graph.getForwardExlusivEdges();
	for(unsigned int e = 0; e < edgeForward.size(); e+=2)
		writeConnectionForward(edgeForward[e],edgeForward[e+1]);

	std::vector<std::shared_ptr<model::Instruction> > edgeBackward = _graph.getBackwardExclusivEdges();
	for(unsigned int e = 0; e < edgeBackward.size(); e+=2)
		writeConnectionBackwards(edgeBackward[e],edgeBackward[e+1]);
}

// **********************************
// *** HEADERS AND FOOTERS

void GraphPrint::openFile()
{
	std::string filePath = ros::package::getPath("reversible_d7pc_interpreter") + "/src/gennerated_code/";
	std::string fileName = "program_graph.dot";
	_outputFile.open(filePath + fileName);

	if (_outputFile.is_open() == false)
	{
		std::cerr << "Failed to open file in GraphPrint" << std::endl;
		exit(-1);
	}
}


void GraphPrint::closeFile()
{
	_outputFile.close();
}



void GraphPrint::writeHeader()
{
	_outputFile << "digraph graphname {";
}


void GraphPrint::writeFooter()
{
	std::string path = ros::package::getPath("reversible_d7pc_interpreter") + "/src/gennerated_code/";
	_outputFile << "\n }" << std::endl;
	std::string input = path + "/program_graph.dot";
	std::string output = path + "/program_graph.png";
	std::string cmd = "dot -Tpng " + input + " -o " + output;
	system(cmd.c_str());
}


// **********************************
// *** SUPPORT AND COLORS

std::string GraphPrint::generateNodeIdentification(std::shared_ptr<model::Instruction> a)
{
	//return a->getID();
	std::stringstream ss;
	ss << a;
	return ss.str();

}

std::string GraphPrint::addQutationMark(std::string str)
{
	return "\"" + str + "\"";
}

std::string GraphPrint::generateColorsManual(std::string str)
{
	if(!str.compare("io")) 			return "/paired12/1";
	if(!str.compare("move")) 		return "/paired12/2";
	if(!str.compare("wait")) 		return "/paired12/3";
	if(!str.compare("grasp")) 		return "/paired12/4";
	if(!str.compare("action")) 		return "/paired12/5";
	if(!str.compare("force mode")) 	return "/paired12/6";
	if(!str.compare("empty linker"))return "/paired12/7";
	if(!str.compare("print"))		return "/paired12/8";
	if(!str.compare("error check")) return "/paired12/9";
	if(!str.compare("unspecified")) return "/paired12/10";
	return generateColorsAutomatic(str);
}

std::string GraphPrint::generateColorsAutomatic(std::string str)
{
	unsigned int hash = 1;
	for(std::string::const_iterator it=str.begin(); it!=str.end(); ++it)
		hash = (hash * (*it)) % 0xFFFFFF;

	unsigned int pair1 =  hash % 0x100;
	unsigned int pair2 = (hash / 0x100) %0x100;
	unsigned int pair3 =  hash / 0x10000;

	unsigned int offset = 0x60;
	unsigned int rem = 0xFF - offset;
	hash  =  offset + pair1 % rem;
	hash += (offset + pair2 % rem)*0x100;
	hash += (offset + pair3 % rem)*0x10000;

	 std::stringstream stream;
	 stream << "#"
	        << std::setfill ('0') << std::setw(6)
	        << std::hex << hash;
	 return  stream.str();
}


// **********************************
// *** NODES

std::string GraphPrint::generateNodeProperties(std::shared_ptr<model::Instruction> a)
{
	bool uncall = a->isUncallLayoutDEBUG();
	bool swapped = a->isSwappedLayoutDEBUG();
	bool neverR = !(a->isReversibleDEBUG());

	std::string ret = " ";

	if(uncall)	ret += "U ";
	if(swapped)	ret += "S";
	if(neverR)	ret += " N";

	return ret;
}

std::string GraphPrint::generateNodeArgument(std::shared_ptr<model::Instruction> a)
{
	std::string f;
	std::string b;
	b += "<BR/>";
	b += "bw: " + a->getArgumentBackward();
	f += "<BR/>";
	f += "fw: " + a->getArgumentForward();

	std::string ret = (a->isSwappedLayoutDEBUG() ? f + b : b + f);

	if(ret.size() < 19)
		return "";

	return ret;
}


void GraphPrint::writeNode(std::shared_ptr<model::Instruction> a)
{
	std::string nodeID = generateNodeIdentification(a);
	std::string labelSeq = a->getID();
//	std::string labelSeq = a->getIDClass().getOrigin();
	std::string labelCmd = a->getType();
	std::string labelPro = generateNodeProperties(a);
	std::string labelSta = generateNodeArgument(a);

	std::string label;
	label  = "label = ";
	label += "<" + labelCmd;
	label += "<BR/><FONT POINT-SIZE=\"10\">" + labelSeq;
	label += labelSta;
	label += "<BR/>" + labelPro;
	label += "</FONT>>";

	std::string group;
	group  = ", group = ";
	group += addQutationMark( a->getIDClass().getGrouping() );

	std::string shape;
	shape  = ", shape=box";
	shape  = "";

	std::string size;
	size  = ", fixedsize=\"false\", width=0.75, height=0.75";
	size  = "";

	std::string color;
	color  = ", style=filled, fillcolor = ";
	color += addQutationMark( generateColorsManual(labelCmd) );

	std::string argument = "[" + label + group + shape + size + color + "]";
	writeNode(nodeID, argument);
}

void GraphPrint::writeNode(std::string node, std::string argument)
{
	_outputFile << "\n\"" << node << "\"" << argument << ";";
}





// **********************************
// *** EDGES

void GraphPrint::writeConnectionDouble(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b)
{
	writeConnectionArgumented(a,b, "[ dir=both, arrowtail=onormal ]");
//	writeConnectionArgumented(a,b);
//	writeConnectionArgumented(b,a, "[ style=dotted, arrowhead=onormal ]");
}

void GraphPrint::writeConnectionForward(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b)
{
	writeConnectionArgumented(a,b);
}

void GraphPrint::writeConnectionBackwards(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b)
{
	writeConnectionArgumented(a,b, "[ arrowhead=onormal ]");
}

void GraphPrint::writeConnectionArgumented(std::shared_ptr<model::Instruction> a, std::shared_ptr<model::Instruction> b, std::string argument)
{
	std::string idA = generateNodeIdentification(a);
	std::string idB = generateNodeIdentification(b);
	writeConnectionArgumented(idA,idB,argument);
}

void GraphPrint::writeConnectionArgumented(std::string a, std::string b, std::string argument)
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

} /* namespace common */
} /* namespace dsl */
