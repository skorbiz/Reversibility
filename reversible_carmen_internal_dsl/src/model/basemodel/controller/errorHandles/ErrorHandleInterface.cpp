/*
 * ErrorHandleInterface.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: josl
 */

#include "ErrorHandleInterface.hpp"

namespace dsl {

ErrorHandleInterface::ErrorHandleInterface() {
	cerr.setPrefix("[error] ");
	cerr.setColorPrefix(dsl::color::RED);
}

ErrorHandleInterface::~ErrorHandleInterface() {
}

} /* namespace dsl */
