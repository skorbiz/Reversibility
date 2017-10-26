/*
 * CodeGen.h
 *
 *  Created on: Jun 9, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PC_INTERPRETER_CODEGEN_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PC_INTERPRETER_CODEGEN_H_

#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

namespace edsl
{

class CodeGen
{

public:
	CodeGen(std::string template_text, std::string output_file);
	virtual ~CodeGen();

	void addReplacementRule(std::string templete_var_name, std::string replacement);
	void addRegexReplacementRule(std::string regex_expresion, std::string replacement);
	void gennerate();

private:
	void openFile();
	void closeFile();

	std::string gennerate(std::string template_text, std::string regex_rule, std::string replacement);
	bool check_output(std::string output);

private:
	std::string _template;
	std::string _regexGenericIdentifier;
	std::vector<std::pair<std::string, std::string> > _replacementRules;
	std::string	_outputFilePath;
	std::ofstream _outputFile;

};

} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PC_INTERPRETER_CODEGEN_H_ */
