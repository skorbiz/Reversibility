/*
 * MessagesError.hpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#ifndef MESSAGESERROR_HPP_
#define MESSAGESERROR_HPP_

#include <../src/model/basemodel/controller/messages/MessageEnvelope.hpp>

namespace dsl
{

class MessagesError : public dsl::MessageEnvelope
{

public:
	MessagesError();
	virtual ~MessagesError();

	int getErrorCount();
	void resetErrorCount();
	void increaseErrorCount();

private:
	int _errorCount;

};

} /* namespace dsl */

#endif /* MESSAGESERROR_HPP_ */
