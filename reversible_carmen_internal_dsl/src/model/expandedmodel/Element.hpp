/*
w * Element.hpp
 *
 *  Created on: Nov 24, 2014
 *      Author: josl
 */

#ifndef ELEMENT_HPP_
#define ELEMENT_HPP_

#include <iostream>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/expandedmodel/intermediateModel/IntermediateModel.hpp>

namespace dsl
{

class Element
{
protected:
	enum class direction
	{
		FORWARD, REVERSE
	};

public:
	Element();
	virtual ~Element();

	void setIsReversible(bool isReversible);
	void setReverseCounterpart(std::shared_ptr<dsl::Element> reversecounterpart);
	std::shared_ptr<dsl::Element> getReverseCounterpart();


public:
	virtual void buildIntermediateModel(direction dir, bool uncallLayout, bool isReversible, dsl::Identification::Ptr name, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel * > & memory) = 0;

protected:
	bool _isReversible;

private:
	std::shared_ptr<dsl::Element> _reverseCounterpart;

};

} /* namespace dsl */

#endif /* ELEMENT_HPP_ */
