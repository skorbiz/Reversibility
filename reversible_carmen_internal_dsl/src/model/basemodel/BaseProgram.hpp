/*
 * BaseProgram.hpp
 *
 *  Created on: May 30, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_BASEPROGRAM_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_BASEPROGRAM_HPP_

#include <../src/model/basemodel/controller/messages/MessageEnvelope.hpp>
#include <../src/model/basemodel/controller/messages/MessagesContainer.hpp>
#include <../src/model/basemodel/Identification.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/basemodel/InstructionGraph.hpp>
#include <../src/model/basemodel/State.hpp>

namespace dsl {

class BaseProgram {
public:

	typedef std::shared_ptr<BaseProgram> Ptr;

	BaseProgram(std::shared_ptr<dsl::Instruction> ins);
	virtual ~BaseProgram();

	std::shared_ptr<dsl::Instruction> getStart();
	std::shared_ptr<dsl::Instruction> getEnd();
	void initInstructions(dsl::MessagesContainer * container);


private:
	void initContainers(dsl::MessagesContainer * container);
	void initStates();
	void initStatesForwardRecursiveLEGACY(dsl::State state, std::shared_ptr<dsl::Instruction> ins);
	void initStatesReverseRecursiveLEGACY(dsl::State state, std::shared_ptr<dsl::Instruction> ins);

	void InitStatesForwardLoop(dsl::State state, std::shared_ptr<dsl::Instruction> ins);
	void InitStatesReverseLoop(dsl::State state, std::shared_ptr<dsl::Instruction> ins);


private:
	dsl::debug::Debug _deb;
	dsl::InstructionGraph _graph;




};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_BASEPROGRAM_HPP_ */
