/*
 * Node.h
 *
 *  Created on: Mar 22, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_BASEMODEL_GRAPH_NODE_H_
#define REVERSIBLE_ASSEMBLY_SRC_BASEMODEL_GRAPH_NODE_H_

#include <iostream>
#include <memory>
//#include <memory.h>
#include "Identification.h"

namespace dsl {
namespace basemodel {
namespace graph {

class Node
{

public:
	Node(Identification id);
	virtual ~Node();

	//Construction functions
	void setNodeNext(std::shared_ptr<Node> node);
	void setNodePrevious(std::shared_ptr<Node> node);
	std::shared_ptr<Node> getNodeNext() const;
	std::shared_ptr<Node> getNodePrevious() const;

	//Other functions
	Identification getIDClass();
	std::string getID();

private:
	Identification _id;
	std::shared_ptr<Node> _nodeNext;
	std::shared_ptr<Node> _nodePrevious;



};

} /* namespace graph */
} /* namespace basemodel */
} /* namespace dsl */

#endif /* REVERSIBLE_ASSEMBLY_SRC_BASEMODEL_GRAPH_NODE_H_ */
