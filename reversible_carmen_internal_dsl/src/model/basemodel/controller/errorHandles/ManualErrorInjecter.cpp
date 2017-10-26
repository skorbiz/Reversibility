/*
 * ManualErrorInjecter.cpp
 *
 *  Created on: Jan 26, 2015
 *      Author: josl
 */

#include "ManualErrorInjecter.hpp"

namespace dsl
{

ManualErrorInjecter::ManualErrorInjecter(dsl::MessagesContainer * container) :
		_isRunning(false),
		_msg(new dsl::MessagesError()),
		_container(container)
//		_loopRate(20.0)
{
}

ManualErrorInjecter::~ManualErrorInjecter()
{
}

void ManualErrorInjecter::run()
{
//	_isRunning = true;
//	while (ros::ok() && _isRunning)
//	{
//		loop();
//	}
}

void ManualErrorInjecter::stop()
{
	_isRunning = false;
}

void ManualErrorInjecter::loop()
{
	std::string input;
	std::cin >> input;
	if(!input.compare("inject") || !input.compare("i"))
	{
		std::cout << "Injected error in controller" << std::endl;
		injectError();
	}
	else if(!input.compare("reset") || !input.compare("r"))
	{
		std::cout << "Reset error count" << std::endl;
		_msg->resetErrorCount();
	}
}

void ManualErrorInjecter::injectError()
{
	_msg->increaseErrorCount();
	_container->pushMessage(_msg);
}

} /* namespace dsl */

