/*
 * IoComparisonMonitored.hpp
 *
 *  Created on: May 23, 2015
 *      Author: josl
 */

#ifndef IOCOMPARISONMONITORED_HPP_
#define IOCOMPARISONMONITORED_HPP_

#include <DigitalIOGeneralInterfaceProxy.hpp>
#include <../src/model/basemodel/argument/quantities/IOPorts.hpp>
#include <../src/model/basemodel/argument/quantities/Switch.hpp>
#include <../src/model/basemodel/condition/ConditionMonitored.hpp>

namespace dsl
{

class IoComparisonMonitored : public ConditionMonitored
{

public:
	IoComparisonMonitored(const dsl::IOPorts & ioports, const dsl::Switch & expectedValues, std::shared_ptr<DigitalIOGeneralInterfaceProxy> DGIP);
	virtual ~IoComparisonMonitored();

	//Construction Functions
	void addArgument(const dsl::IOPorts & arg);
	void addArgument(const dsl::Switch & arg);

	//Overwritten functions
	bool  evaluate();

	private:
		std::shared_ptr<DigitalIOGeneralInterfaceProxy> _DGIP;
		std::vector<int8_t> _ioports;
		std::vector<uint8_t> _expectedValues;

};

} /* namespace dsl */

#endif /* IOCOMPARISONMONITORED_HPP_ */


