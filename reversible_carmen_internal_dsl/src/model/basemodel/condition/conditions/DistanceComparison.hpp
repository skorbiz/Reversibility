/*
 * DistanceComparison.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#ifndef DISTANCECOMPARISON_HPP_
#define DISTANCECOMPARISON_HPP_

#include <../src/model/basemodel/condition/ConditionMonitored.hpp>

namespace dsl {

class DistanceComparison : public ConditionMonitored
{

public:
	DistanceComparison();
	virtual ~DistanceComparison();
};



} /* namespace dsl */

#endif /* DISTANCECOMPARISON_HPP_ */
