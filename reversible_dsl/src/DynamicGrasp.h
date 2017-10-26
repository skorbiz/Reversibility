/*
 * DynamicGrasp.h
 *
 *  Created on: Jan 24, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_DYNAMICGRASP_H_
#define D7PC_ROS_REVERSIBLE_DSL_SRC_DYNAMICGRASP_H_
#include <rw/math/Transform3D.hpp>


class DynamicGrasp
{

public:
	DynamicGrasp(rw::math::Transform3D<> baseTcamera);
	virtual ~DynamicGrasp();

	bool update(rw::math::Transform3D<> cameraTobject);

	rw::math::Transform3D<> baseTtool;
	rw::math::Transform3D<> baseTtoolRetract;
	rw::math::Transform3D<> toolTpart;

private:
	rw::math::Transform3D<> calc_baseTpart(const rw::math::Transform3D<> camTpart) const;
	rw::math::Transform3D<> calc_baseTtool(const rw::math::Transform3D<> baseTpart) const;
	rw::math::Transform3D<> calc_baseTtoolRetract(const rw::math::Transform3D<> baseTtool) const;
	rw::math::Transform3D<> calc_toolTpart(const rw::math::Transform3D<> baseTtool, const rw::math::Transform3D<> baseTpart) const;

	void to_screen(std::string t_name, const rw::math::Transform3D<> t) const;

private:
	const rw::math::Transform3D<> baseTcamera;


};

#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_DYNAMICGRASP_H_ */


