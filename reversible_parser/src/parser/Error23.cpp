/*
 * Error23.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: josl
 */

#include <parser/Error23.h>
#include <iostream>

namespace edsl {
namespace parser {

Error23::Error23() :
	line(-1)
{
}

Error23::Error23(std::string msg, int line) :
	message(msg),
	line(line)
{
}

Error23::Error23(std::string msg, int line, std::shared_ptr<Error23> suberror) :
	message(msg),
	line(line),
	suberror(suberror)
{
}


Error23::~Error23()
{
}

void Error23::to_screen()
{
	std::cout << "Error23: pass failed on line " << line << " with msg: " << message << std::endl;
	if(suberror != nullptr)
		suberror->to_screen();

}

} /* namespace parser */
} /* namespace edsl */
