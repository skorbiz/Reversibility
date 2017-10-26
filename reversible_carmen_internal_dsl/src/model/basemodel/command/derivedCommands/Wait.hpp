/*
 * Wait.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: josl
 */

#ifndef WAIT_HPP_
#define WAIT_HPP_

#include <ros/ros.h>
#include <../src/model/basemodel/command/CommandExecutable.hpp>

namespace dsl {

class Wait : public CommandExecutable
{

public:
	Wait();
	Wait(double time);
	virtual ~Wait();

	virtual void execute();
	virtual void executeBackwards();
	virtual std::string getType();

private:
	double time;

};

} /* namespace dsl */

#endif /* WAIT_HPP_ */
