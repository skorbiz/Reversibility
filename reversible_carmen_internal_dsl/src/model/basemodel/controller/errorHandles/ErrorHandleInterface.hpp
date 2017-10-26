/*
 * ErrorHandleInterface.hpp
 *
 *  Created on: Apr 8, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_ERRORHANDLES_ERRORHANDLEINTERFACE_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_ERRORHANDLES_ERRORHANDLEINTERFACE_HPP_

#include <color/colors.hpp>
#include <debug/Debug.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/basemodel/controller/messages/MessagesError.hpp>
namespace dsl
{

class ErrorHandleInterface
{
public:
	ErrorHandleInterface();
	virtual ~ErrorHandleInterface();

	virtual void update(dsl::MessagesError & msg) = 0;
	virtual void update(std::shared_ptr<dsl::Instruction> ins) = 0;
	virtual bool isInErrorMode() = 0;

protected:
	dsl::debug::Debug cerr;

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_ERRORHANDLES_ERRORHANDLEINTERFACE_HPP_ */
