/*
 * D7pcString2Variable.h
 *
 *  Created on: Jun 9, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_GENNERATED_CODE_D7PCSTRING2VARIABLE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_GENNERATED_CODE_D7PCSTRING2VARIABLE_H_

#include <../../reversible_dsl/src/D7cpWorkcell.h>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <memory>

namespace edsl {

class D7pcString2Variable
{

public:
	D7pcString2Variable();
	virtual ~D7pcString2Variable();

	std::shared_ptr<rw::kinematics::Frame> toFrame(std::string name);
	rw::math::Q toQ(std::string name);

private:
	std::shared_ptr<D7cpWorkcell> wc;

};

} /* namespace edsl */
#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_GENNERATED_CODE_D7PCSTRING2VARIABLE_H_ */
