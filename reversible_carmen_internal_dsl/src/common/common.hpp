/*
 * common.hpp
 *
 *  Created on: Feb 10, 2015
 *      Author: josl
 */

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <ros/ros.h>
#include <ros/package.h>

namespace dsl {
namespace common {

std::string getPath2sduMagic();
std::string getPath2dsl();
std::string getPath2trunk();
std::string getPath2dslOutputFolder();
std::string getPath2dropbox();


} /* namespace common */
} /* namespace dsl */

#endif /* COMMON_HPP_ */
