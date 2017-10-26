/*
 * TestIntActionDataExtraction.hpp
 *
 *  Created on: Jul 5, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_PROGRAMS_TESTINTACTIONDATAEXTRACTION_HPP_
#define REVERSIBLE_DSL_SRC_PROGRAMS_TESTINTACTIONDATAEXTRACTION_HPP_

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <rw/math/Q.hpp>
#include <rw/trajectory.hpp>

#include <../src/model/basemodel/command/derivedCommands/ActionIKSolver.hpp>

#include <LearningProxy.hpp>




namespace dsl {
namespace intelligentmove {

class TestIntActionDataExtraction {
public:
	TestIntActionDataExtraction();
	virtual ~TestIntActionDataExtraction();

	void connectUR();
	void connectIKSolver();
	void connectLearningInterface();


	rw::trajectory::TimedQPath trajectoryLoad();
	std::vector<rw::math::Q> convertTrajectory(rw::trajectory::TimedQPath trajectory, rw::math::Q qStart);
	std::vector<rw::math::Q> getOffsetTrajectory(rw::math::Transform3D<double> offset, rw::math::Q qStart);




private:
	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<rwhw::URCallBackInterface> _ur;
	std::shared_ptr<LearningProxy> _learningInterface;
	std::shared_ptr<ActionIKSolver> _ikSolver;


};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_PROGRAMS_TESTINTACTIONDATAEXTRACTION_HPP_ */
