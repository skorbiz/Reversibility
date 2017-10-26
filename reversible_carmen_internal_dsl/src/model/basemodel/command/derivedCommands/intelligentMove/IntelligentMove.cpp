/*
 * IntelligentMove.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: josl
 */

#include "IntelligentMove.hpp"
#include "MinimumTresholdCalculator.hpp"
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.hpp>

namespace dsl {
namespace intelligentmove {

IntelligentMove::IntelligentMove(std::shared_ptr<rwhw::URCallBackInterface> urc,
								 std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt,
								 rw::math::Q qStart,
								 rw::math::Q qEnd,
								 RecordDataset & dataset)
	: IntelligentTrajectory(urc, urrt, createTrajectory(qStart,qEnd), dataset.getRecordsWith(qStart, qEnd))
{
}


IntelligentMove::~IntelligentMove()
{
}

std::vector<rw::math::Q> IntelligentMove::createTrajectory(rw::math::Q qStart, rw::math::Q qEnd)
{
	std::vector<rw::math::Q> res;
	res.push_back(qStart);
	res.push_back(qEnd);
	return res;
}


} /* namespace intelligentmove */
} /* namespace dsl */
