/*
 * RegexIntepreter.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#include "RegexIntepreter.h"
#include <regex>
#include <boost/regex.hpp>

RegexIntepreter::RegexIntepreter()
{
}

RegexIntepreter::~RegexIntepreter()
{
}


std::string RegexIntepreter::extract_command(std::string input)
{
	std::vector<size_t> positions;
	size_t pos = input.find(":", 0);
	while(pos != std::string::npos)
	{
	    positions.push_back(pos);
	    pos = input.find(":",pos+1);
	}
	positions.push_back(pos);

	if(positions.size() > 2)
		return "error : string contains multiple instances of ':'";


    const std::string nothing = "" ;
    std::string output = input.substr(0, positions[0]);
    output = boost::regex_replace( output, boost::regex( "^\\s+" ), nothing ) ; //Left trim
    output = boost::regex_replace( output, boost::regex( "\\s+$" ), nothing ) ; //Right trim

	return output;
}


std::string RegexIntepreter::extract_args(std::string input)
{
	size_t pos = input.find(":", 0);

	if(pos == std::string::npos)
		return "";

    const std::string nothing = "" ;
    std::string output = input.substr(pos+1, input.size());
    output = boost::regex_replace( output, boost::regex( "^\\s+" ), nothing ) ; //Left trim
    output = boost::regex_replace( output, boost::regex( "\\s+$" ), nothing ) ; //Right trim

	return output;
}

std::vector<std::string> RegexIntepreter::extract_args_list(std::string input)
{
	std::string args = extract_args(input);
	std::vector<std::string> args_list;

	boost::regex rgx("\\s+");
	boost::sregex_token_iterator iter(args.begin(),	args.end(), rgx, -1);
	boost::sregex_token_iterator end;
	for ( ; iter != end; ++iter)
	    args_list.push_back(*iter);

	return args_list;
}





