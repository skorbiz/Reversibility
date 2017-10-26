/*
 * ElementSequenceNested.hpp
 *
 *  Created on: Dec 24, 2014
 *      Author: josl
 */

#ifndef ELEMENTSEQUENCENESTED_HPP_
#define ELEMENTSEQUENCENESTED_HPP_

#include <iostream>
#include <../src/model/expandedmodel/Element.hpp>
#include <../src/model/expandedmodel/Sequence.hpp>

namespace dsl
{

class ElementSequenceNested : public Element
{

public:
	ElementSequenceNested(std::shared_ptr<dsl::Sequence> nestedSequence, bool reverseLayout = false, bool uncallLayout = false);
	virtual ~ElementSequenceNested();

	void buildIntermediateModel(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory);

private:
	Element::direction switchDirection(direction dir);


private:
	std::shared_ptr<dsl::Sequence> _nestedSequence;
	bool _reverseLayout;
	bool _uncallLayout;
};

} /* namespace dsl */

#endif /* ELEMENTSEQUENCENESTED_HPP_ */
