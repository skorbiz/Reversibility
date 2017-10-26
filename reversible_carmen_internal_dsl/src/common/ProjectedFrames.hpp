/*
 * ProjectedFrames.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_COMMON_PROJECTEDFRAMES_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_COMMON_PROJECTEDFRAMES_HPP_

#include <iostream>
#include <rw/math.hpp>
#include <../src/common/WorkcellModel.hpp>

namespace dsl {
namespace common {

class ProjectedFrames {
public:
	ProjectedFrames(dsl::common::WorkcellModel wc);
	virtual ~ProjectedFrames();

	rw::math::Transform3D<> getTransformBaseTNew(WorkcellModel::frame aframe, rw::math::Vector3D<> moveDirection);
	rw::math::Transform3D<> getTransformBaseTProjected(const rw::math::Transform3D<> & baseTselected);

private:
	dsl::common::WorkcellModel _wc;

};

} /* namespace common */
} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_COMMON_PROJECTEDFRAMES_HPP_ */
