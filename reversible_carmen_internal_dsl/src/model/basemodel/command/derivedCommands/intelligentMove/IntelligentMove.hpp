/*
 * IntelligentMove.hpp
 *
 *  Created on: Mar 1, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTMOVE_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTMOVE_HPP_


#include <rw/math/Q.hpp>

#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include "IntelligentTrajectory.hpp"
#include "RecordDataset.hpp"


namespace dsl {
namespace intelligentmove {

class IntelligentMove : public IntelligentTrajectory
{

public:
	IntelligentMove(std::shared_ptr<rwhw::URCallBackInterface> urc, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, rw::math::Q qStart, rw::math::Q qEnd, RecordDataset & dataset);
	virtual ~IntelligentMove();

private:
	static std::vector<rw::math::Q> createTrajectory(rw::math::Q qStart, rw::math::Q qEnd);

};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTMOVE_HPP_ */
