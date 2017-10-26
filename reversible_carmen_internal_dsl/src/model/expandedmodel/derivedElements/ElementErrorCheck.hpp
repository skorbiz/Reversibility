/*
 * ElementErrorCheck.hpp
 *
 *  Created on: Jan 13, 2015
 *      Author: josl
 */

#ifndef ELEMENTERRORCHECK_HPP_
#define ELEMENTERRORCHECK_HPP_

#include <iostream>
#include <../src/model/basemodel/condition/Condition.hpp>
#include <../src/model/basemodel/condition/ConditionInstant.hpp>
#include <../src/model/basemodel/condition/ErrorCheck.hpp>
#include <../src/model/basemodel/controller/messages/MessagesError.hpp>
#include <../src/model/expandedmodel/Element.hpp>
#include <../src/model/expandedmodel/intermediateModel/IntermediateModelIns.hpp>

namespace dsl
{

class ElementErrorCheck : public Element
{

public:
	ElementErrorCheck(dsl::Condition * condition);
	virtual ~ElementErrorCheck();

	void buildIntermediateModel(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory);

private:
	std::shared_ptr<dsl::ErrorCheck> _insErrorCheck;
	dsl::Condition * _condition;
	dsl::MessagesError * _msgError;
};

} /* namespace dsl */

#endif /* ELEMENTERRORCHECK_HPP_ */
