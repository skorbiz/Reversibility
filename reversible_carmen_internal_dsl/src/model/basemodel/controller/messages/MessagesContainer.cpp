/*
 * MessagesContainer.cpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#include "MessagesContainer.hpp"

namespace dsl
{

MessagesContainer::MessagesContainer()
{
}

MessagesContainer::~MessagesContainer()
{
}

void MessagesContainer::pushMessage(dsl::MessageEnvelope * msg)
{
	_messages.push_back(msg);
}

dsl::MessageEnvelope * MessagesContainer::popMessage()
{
	//TODO obs. empty messages are not destructed

	if(containerIsEmpty())
		{std::cerr << "Tryed to get messege from empty container" << std::endl; exit(-1);}

	dsl::MessageEnvelope * ret = _messages[0];
	_messages.erase(_messages.begin());
	return ret;
}

bool MessagesContainer::containerIsEmpty()
{
//	std::cout << _messages.size() << std::endl;
//	if(_messages.empty())
//		std::cout << "true" << std::endl;
//	else
//		std::cout << "false" << std::endl;
	return _messages.empty();
}

} /* namespace dsl */
