/*
 * common.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: josl
 */

#include "common.hpp"

namespace dsl{
namespace common{

std::string getPath2sduMagic()
{
	return ros::package::getPath("reversible_dsl_v1");
}

std::string getPath2dsl()
{
	return getPath2sduMagic() + "/src";
}

std::string getPath2trunk()
{
	return getPath2sduMagic() + "/../../../..";
}

std::string getPath2dslOutputFolder()
{
	return getPath2trunk() + "/dsl/outputs";
}

std::string getPath2dropbox()
{
	return getPath2trunk() + "/../../../Dropbox";
}


} /* namespace common */
} /* namespace dsl */
