/*
 * ElementErrorCheckMonitoredStart.hpp
 *
 *  Created on: May 23, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_EXPANDEDMODEL_DERIVEDELEMENTS_ELEMENTERRORCHECKMONITOREDSTART_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_EXPANDEDMODEL_DERIVEDELEMENTS_ELEMENTERRORCHECKMONITOREDSTART_HPP_

#include <iostream>
#include <../src/model/basemodel/condition/ConditionMonitored.hpp>
#include <../src/model/basemodel/condition/ErrorCheckMonitoredStart.hpp>
#include <../src/model/expandedmodel/Element.hpp>
#include <../src/model/expandedmodel/intermediateModel/IntermediateModelIns.hpp>

namespace dsl
{

class ElementErrorCheckMonitoredStart : public Element
{
public:
	ElementErrorCheckMonitoredStart(dsl::ConditionMonitored * condition);
	virtual ~ElementErrorCheckMonitoredStart();
	void buildIntermediateModel(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory);

private:
	std::shared_ptr<dsl::ErrorCheckMonitoredStart> _insErrorCheckMS;
	dsl::ConditionMonitored * _condition;
};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_EXPANDEDMODEL_DERIVEDELEMENTS_ELEMENTERRORCHECKMONITOREDSTART_HPP_ */


