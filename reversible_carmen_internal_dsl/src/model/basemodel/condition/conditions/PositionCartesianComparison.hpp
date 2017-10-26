/*
 * PositionCartesianComparison.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#ifndef POSITIONCARTESIANCOMPARISON_HPP_
#define POSITIONCARTESIANCOMPARISON_HPP_

#include <rw/rw.hpp>
#include <rw/math.hpp>

#include <RobotGeneralInterfaceProxy.hpp>

#include <../src/model/basemodel/argument/quantities/JointConfiguration.hpp>
#include <../src/model/basemodel/condition/ConditionInstant.hpp>
#include <debug/Debug.hpp>
#include <color/colors.hpp>
#include <../src/common/WorkcellModel.hpp>

namespace dsl
{

class PositionCartesianComparison : public ConditionInstant
{
public:
	PositionCartesianComparison(const dsl::JointConfiguration & q, double epsilon, std::shared_ptr<RobotGeneralInterfaceProxy> RGIP);
	PositionCartesianComparison(rw::math::Q q, double epsilon, std::shared_ptr<RobotGeneralInterfaceProxy> RGIP);
	virtual ~PositionCartesianComparison();

	//Overwritten functions
	bool evaluate();
	void stateUpdate(dsl::State const& state);

private:
	//Construction Functions
	void addArgument(const dsl::JointConfiguration & q);
	void errorVisualizer(double error);

private:
	std::shared_ptr<RobotGeneralInterfaceProxy> _RGIP;
	rw::math::Q _qExpected;
	double _epsilon;

	dsl::debug::Debug dco;



};

} /* namespace dsl */

#endif /* POSITIONCARTESIANCOMPARISON_HPP_ */
