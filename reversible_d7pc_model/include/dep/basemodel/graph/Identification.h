/*
 * Identification.h
 *
 *  Created on: Mar 22, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_BASEMODEL_GRAPH_IDENTIFICATION_H_
#define REVERSIBLE_ASSEMBLY_SRC_BASEMODEL_GRAPH_IDENTIFICATION_H_

#include <string>

namespace dsl {
namespace basemodel {
namespace graph {

class Identification
{

public:
	Identification(std::string sequence, int instance, std::string number, int depth);
	Identification(std::string sequence, int instance, int number, int depth);
	virtual ~Identification();

	std::string getOrigin() const;
	std::string getInstance() const;
	std::string getGrouping() const;
	std::string getNumber() const;
	int			getDepth() const;

	std::string toString() const;
	operator std::string() const;
	friend std::ostream& operator<<(std::ostream& os, const Identification& id);


private:
	std::string _origin;
	std::string _instance;
	std::string _number;
	int	_depth;

};

} /* namespace graph */
} /* namespace basemodel */
} /* namespace dsl */

#endif /* REVERSIBLE_ASSEMBLY_SRC_BASEMODEL_GRAPH_IDENTIFICATION_H_ */
