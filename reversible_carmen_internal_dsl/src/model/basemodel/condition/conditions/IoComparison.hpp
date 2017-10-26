/*
 * IoComparison.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#ifndef IOCOMPARISON_HPP_
#define IOCOMPARISON_HPP_

#include <DigitalIOGeneralInterfaceProxy.hpp>
#include <../src/model/basemodel/argument/quantities/IOPorts.hpp>
#include <../src/model/basemodel/argument/quantities/Switch.hpp>
#include <../src/model/basemodel/condition/ConditionInstant.hpp>

namespace dsl
{

class IoComparison : public ConditionInstant
{

public:
	IoComparison(const dsl::IOPorts & ioports, const dsl::Switch & expectedValues, std::shared_ptr<DigitalIOGeneralInterfaceProxy> DGIP);
	virtual ~IoComparison();

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

#endif /* IOCOMPARISON_HPP_ */


