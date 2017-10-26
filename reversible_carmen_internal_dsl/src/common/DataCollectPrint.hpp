/*
 * DataCollectPrint.hpp
 *
 *  Created on: Jan 26, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_COMMON_DEBUG_DATACOLLECTPRINT_HPP_
#define REVERSIBLE_DSL_SRC_COMMON_DEBUG_DATACOLLECTPRINT_HPP_

#include <map>
#include <vector>

namespace dsl {
namespace common {

class DataCollectPrint
{

public:
	DataCollectPrint();
	virtual ~DataCollectPrint();
	void add(std::string id, double val);

private:
	std::map<std::string, std::vector<double> > memory;

};

} /* namespace common */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_COMMON_DEBUG_DATACOLLECTPRINT_HPP_ */
