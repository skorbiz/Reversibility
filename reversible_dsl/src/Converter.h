/*
 * Converter.h
 *
 *  Created on: Mar 10, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_CONVERTER_H_
#define REVERSIBLE_DSL_SRC_CONVERTER_H_

#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>

class Converter
{

public:
	Converter(rw::models::WorkCell::Ptr wc,
			  rw::models::Device::Ptr device,
			  std::shared_ptr<rw::kinematics::Frame> toolFrame);
	virtual ~Converter();

	rw::math::Transform3D<> frameTframe(std::shared_ptr<rw::kinematics::Frame> from, std::shared_ptr<rw::kinematics::Frame> to, const rw::kinematics::State& state) const;
	rw::math::Transform3D<> toBaseTtool(rw::math::Q q, const rw::kinematics::State state_input) const;
	rw::math::Transform3D<> toBaseTtool(const rw::math::Transform3D<> toolTobject, const std::shared_ptr<rw::kinematics::Frame> objTarget, const rw::kinematics::State state) const;

	rw::math::Transform3D<> toBaseTtool( const rw::kinematics::State state) const;
	rw::math::Q toQ( const rw::kinematics::State state) const;

	rw::math::Transform3D<> rotated_deg(double rz, double ry, double rx, rw::math::Transform3D<> input);

	void toScreen(std::string name, rw::math::Transform3D<> transform) const;

private:
	rw::models::WorkCell::Ptr wc;
	rw::models::Device::Ptr device;
	std::shared_ptr<rw::kinematics::Frame> toolFrame;


};

#endif /* REVERSIBLE_DSL_SRC_CONVERTER_H_ */
