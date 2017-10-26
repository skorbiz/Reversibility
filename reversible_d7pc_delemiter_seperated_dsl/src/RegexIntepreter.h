/*
 * RegexIntepreter.h
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_REGEXINTEPRETER_H_
#define REVERSIBLE_ASSEMBLY_SRC_REGEXINTEPRETER_H_

#include <iostream>
#include <vector>

class RegexIntepreter {
public:
	RegexIntepreter();
	virtual ~RegexIntepreter();

	static std::string extract_command(std::string input);
	static std::string extract_args(std::string input);
	static std::vector<std::string> extract_args_list(std::string input);





};

#endif /* REVERSIBLE_ASSEMBLY_SRC_REGEXINTEPRETER_H_ */
