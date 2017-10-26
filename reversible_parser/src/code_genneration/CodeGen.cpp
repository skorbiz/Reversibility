/*
 * CodeGen.cpp
 *
 *  Created on: Jun 9, 2017
 *      Author: josl
 */

#include <code_genneration/CodeGen.h>
#include <cstdlib>
#include <fstream>
#include <boost/regex.hpp>

namespace edsl {

CodeGen::CodeGen(std::string template_text, std::string output_file) :
		_template(template_text),
		_regexGenericIdentifier("\\$\\$(.)*?(?=\\s)"),
		_outputFilePath(output_file)
{
}

CodeGen::~CodeGen()
{
}

void CodeGen::addReplacementRule(std::string templete_var_name, std::string replacement)
{
	std::string regex = "\\$\\$(" + templete_var_name + ")*?(?=\\s)";
	addRegexReplacementRule(regex, replacement);
}

void CodeGen::addRegexReplacementRule(std::string regex_expresion, std::string replacement)
{
	std::pair<std::string,std::string> rule(regex_expresion, replacement);
	_replacementRules.push_back(rule);
}

void CodeGen::gennerate()
{
	openFile();

	std::string output = _template;

	for(auto rule : _replacementRules)
		output = gennerate(output, rule.first, rule.second);

	check_output(output);

	_outputFile << output << std::endl;
	closeFile();
}


void CodeGen::openFile()
{
	_outputFile.open(_outputFilePath);

	if (_outputFile.is_open() == false)
	{
		std::cerr << "Failed to open files in CodeGen" << std::endl;
		exit(-1);
	}

}

void CodeGen::closeFile()
{
	_outputFile.close();
}

std::string CodeGen::gennerate(std::string template_text, std::string regex_rule, std::string replacement)
{
	boost::regex expr(regex_rule);
	std::string output = boost::regex_replace(template_text, expr, replacement);
	return output;
}

bool CodeGen::check_output(std::string output)
{
	boost::regex expr{_regexGenericIdentifier};
	boost::smatch what;
	if (boost::regex_search(output, what, expr))
	{
		std::cout << "Warning: " << __PRETTY_FUNCTION__ << " failed to generate file - found unused variable" << std::endl;
		std::cout << "Found: " << std::endl;
		std::cout << what[0] << '\n';
		std::cout << what[1] << "_" << what[2] << '\n';
		assert(false);
		return false;
	}
	return true;
}



// **********************************
// *** support


} /* namespace edsl */
