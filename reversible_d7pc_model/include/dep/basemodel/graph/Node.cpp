/*
 * Node.cpp
 *
 *  Created on: Mar 22, 2017
 *      Author: josl
 */

#include "Node.h"
#include <string>

namespace dsl {
namespace basemodel {
namespace graph {

Node::Node(Identification id) :
		 _id(id)
		,_nodeNext(nullptr)
		,_nodePrevious(nullptr)
{
}

Node::~Node()
{
}

void Node::setNodeNext(std::shared_ptr<Node> ins)
{
	this->_nodeNext = ins;
}

void Node::setNodePrevious(std::shared_ptr<Node> ins)
{
	this->_nodePrevious= ins;
}

std::shared_ptr<Node> Node::getNodeNext() const
{
	return this->_nodeNext;
}

std::shared_ptr<Node> Node::getNodePrevious() const
{
	return this->_nodePrevious;
}

std::string Node::getID()
{
	return _id.toString();
}

Identification Node::getIDClass()
{
	return _id;
}

} /* namespace graph */
} /* namespace basemodel */
} /* namespace dsl */

