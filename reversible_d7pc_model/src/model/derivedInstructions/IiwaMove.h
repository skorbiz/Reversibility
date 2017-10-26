/*
 * IiwaMove.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_IIWAMOVE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_IIWAMOVE_H_

#include <model/ExecutionResult23.h>
#include <model/InstructionExecutable.h>
#include <string>

namespace model {

class IiwaMove  : public InstructionExecutable
{

public:
	IiwaMove(std::string text, std::string text_from);
	virtual ~IiwaMove();

	virtual ExecutionResult23 execute();
	virtual ExecutionResult23 executeBackwards();
	std::string getType();

private:
	std::string _text;
	std::string _textFrom;

};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_IIWAMOVE_H_ */
