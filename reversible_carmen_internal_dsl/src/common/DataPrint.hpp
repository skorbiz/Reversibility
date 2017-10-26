/*
 * DataPrint.hpp
 *
 *  Created on: Jan 26, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_COMMON_DATAPRINT_HPP_
#define REVERSIBLE_DSL_SRC_COMMON_DATAPRINT_HPP_

#include <iostream>
#include <fstream>
#include <string>

#include <rw/math/Q.hpp>


namespace dsl {
namespace common {

class DataPrint {
public:
	DataPrint(std::string filePath, std::string fileName);
	virtual ~DataPrint();

	void writeForceHeader();
	void writeForceExtendedHeader();

	void writeForce(const rw::math::Q force);
	void writeForceExtended(const rw::math::Q force, const rw::math::Q q, const rw::math::Q dq, double time, double speed);

	std::string fileName;
	std::string filePath;
	std::ofstream _outputFile;

};

} /* namespace common */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_COMMON_DATAPRINT_HPP_ */
