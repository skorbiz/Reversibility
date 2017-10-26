/*
 * ErrorHandleDeterministic.hpp
 *
 *  Created on: Apr 8, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_ERRORHANDLES_ERRORHANDLEDETERMINISTIC_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_ERRORHANDLES_ERRORHANDLEDETERMINISTIC_HPP_

#include <../src/model/basemodel/controller/errorHandles/ErrorHandleInterface.hpp>

namespace dsl {

class ErrorHandleDeterministic : public ErrorHandleInterface
{

public:
	ErrorHandleDeterministic();
	virtual ~ErrorHandleDeterministic();

	void update(dsl::MessagesError & msg);
	void update(std::shared_ptr<dsl::Instruction> ins);
	bool isInErrorMode();

private:
	bool _inErrorMode;
	int _numberOfSequencesLeftToHandleError;

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_ERRORHANDLES_ERRORHANDLEDETERMINISTIC_HPP_ */
