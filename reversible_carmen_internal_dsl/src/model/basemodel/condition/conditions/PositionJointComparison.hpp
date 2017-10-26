/*
 * PositionJointComparison.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#ifndef POSITIONJOINTCOMPARISON_HPP_
#define POSITIONJOINTCOMPARISON_HPP_

#include <rw/math.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <../src/model/basemodel/argument/quantities/JointConfiguration.hpp>
#include <../src/model/basemodel/condition/ConditionInstant.hpp>
#include <debug/Debug.hpp>
#include <color/colors.hpp>
namespace dsl
{

class PositionJointComparison : public ConditionInstant
{

public:
	PositionJointComparison(const dsl::JointConfiguration & q, double epsilon, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt);
	PositionJointComparison(rw::math::Q q, double epsilon, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt);
	virtual ~PositionJointComparison();

	//Overwritten functions
	bool evaluate();
	void stateUpdate(dsl::State const& state);


private:
	//Construction Functions
	void addArgument(const dsl::JointConfiguration & q);
	void errorVisualizer(double error);

private:
	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	rw::math::Q _qExpected;
	double _epsilon;

	dsl::debug::Debug dco;

};

} /* namespace dsl */

#endif /* POSITIONJOINTCOMPARISON_HPP_ */
