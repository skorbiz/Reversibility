/*
 * JointConfiguration.hpp
 *
 *  Created on: Nov 10, 2014
 *      Author: josl
 */

#ifndef JOINTCONFIGURATION_HPP_
#define JOINTCONFIGURATION_HPP_

#include <iostream>
#include <vector>
#include <rw/math/Q.hpp>
#include <../src/model/basemodel/argument/Argument.hpp>
#include <../src/model/basemodel/argument/Datatype.hpp>

namespace dsl {

class JointConfiguration : public Argument, public Datatype<rw::math::Q>
{
public:
	JointConfiguration();
	JointConfiguration(rw::math::Q value);
	JointConfiguration(double q0, double q1, double q2, double q3, double q4, double q5);
	JointConfiguration(std::initializer_list<double> q);

	virtual ~JointConfiguration();

	friend std::ostream& operator<<(std::ostream& os, const JointConfiguration& q);

	std::string name;
};

} /* namespace dsl */


#endif /* JOINTCONFIGURATION_HPP_ */
