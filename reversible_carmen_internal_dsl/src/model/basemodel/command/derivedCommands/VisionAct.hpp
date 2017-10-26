/*
 * VisionAct.hpp
 *
 *  Created on: Sep 7, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_VISIONACT_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_VISIONACT_HPP_

#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <rw/math.hpp>
#include <RobotGeneralInterfaceProxy.hpp>
#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/common/WorkcellModel.hpp>

namespace dsl
{

class VisionAct : public dsl::CommandExecutable
{

public:
	VisionAct(std::shared_ptr<RobotGeneralInterfaceProxy> RGIP);
	virtual ~VisionAct();
	void execute();

private:
	void calculate();
	rw::math::Q inverseKinimatic(rw::math::Q init, rw::math::Transform3D<> target);
	bool wayToSort(int i, int j);
	bool myComparison(rw::math::Q i, rw::math::Q j);

	rw::math::Transform3D<> fetchWorldT2Base();
	rw::math::Transform3D<> fetchUrT2Tcp();
	rw::math::Transform3D<> fetchObjectT2Above();
	rw::math::Transform3D<> fetchCameraT2Object();
	rw::math::Transform3D<> fetchWorldT2Camera();

private:
	std::shared_ptr<RobotGeneralInterfaceProxy> _RGIP;
	dsl::common::WorkcellModel _wc;
	rw::math::Q _qInitial;
	rw::math::Q _qAbove;
	rw::math::Q _qAt;
	double _speed;
	double _acceleration;


};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_VISIONACT_HPP_ */
