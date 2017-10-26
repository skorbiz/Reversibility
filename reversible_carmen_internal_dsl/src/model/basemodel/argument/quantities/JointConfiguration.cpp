/*
 * JointConfiguration.cpp
 *
 *  Created on: Nov 10, 2014
 *      Author: josl
 */

#include "JointConfiguration.hpp"

namespace dsl {

JointConfiguration::JointConfiguration()
{

}

JointConfiguration::~JointConfiguration()
{
}


JointConfiguration::JointConfiguration(rw::math::Q value) : Datatype(value)
{
}

JointConfiguration::JointConfiguration(double q0, double q1, double q2, double q3, double q4, double q5) : Datatype(rw::math::Q(6,q0,q1,q2,q3,q4,q5))
{
}

JointConfiguration::JointConfiguration(std::initializer_list<double> q) : Datatype(rw::math::Q(std::vector<double>(q)))
{
//
//	for( auto elem : ioports )
//	{
//		get().push_back(elem);
//	}
}


std::ostream& operator<<(std::ostream& os, const JointConfiguration& q)
{
    os << "JointConfiguration(" << q.get() << ")";
    return os;
}


} /* namespace dsl */
