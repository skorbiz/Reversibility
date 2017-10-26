/*
 * MessagesContainer.hpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#ifndef MESSAGESCONTAINER_HPP_
#define MESSAGESCONTAINER_HPP_

#include <iostream>
#include <vector>
#include <../src/model/basemodel/controller/messages/MessageEnvelope.hpp>

namespace dsl
{

class MessagesContainer
{
public:
	MessagesContainer();
	virtual ~MessagesContainer();

public:
	void pushMessage(dsl::MessageEnvelope * msg);
	dsl::MessageEnvelope * popMessage();
	bool containerIsEmpty();

private:
	std::vector<dsl::MessageEnvelope*> _messages;

};

} /* namespace dsl */

#endif /* MESSAGESCONTAINER_HPP_ */
