/*
 * ASTNode.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <assert.h>
#include <ast/ASTNode.h>
#include <lexer/Token.h>
#include <iostream>

namespace edsl {
namespace abstract_syntax_tree {

ASTNode::ASTNode()
{
}

ASTNode::~ASTNode()
{
}

std::ostream& operator<<(std::ostream& os, const ASTNode& t)
{
	os << "ASTNode(" <<t.type << "," << t.getContent()<<")";
	return os;
}

size_t ASTNode::size() const
{
	return abstractSyntaxTreeChilds.size();
}

std::string ASTNode::getContent() const
{
	if(content != nullptr)
		return content->payload;
	return "";

}

std::string ASTNode::getContent(unsigned int child_index) const
{
	if(abstractSyntaxTreeChilds.size() <= child_index)
		std::cout << "Warning: ASTNode::getContent(unsigned int) got index " << child_index << " which is larger then the number of childs: " <<  abstractSyntaxTreeChilds.size() << std::endl;
	assert(abstractSyntaxTreeChilds.size() > child_index );
	return abstractSyntaxTreeChilds[child_index]->getContent();
}

std::string ASTNode::getType() const
{
	return type;
}

std::string ASTNode::getType(unsigned int child_index) const
{
	if(abstractSyntaxTreeChilds.size() <= child_index)
		std::cout << "Warning: ASTNode::getType(unsigned int) got index " << child_index << " which is larger then the number of childs: " <<  abstractSyntaxTreeChilds.size() << std::endl;
	assert(abstractSyntaxTreeChilds.size() > child_index );
	return abstractSyntaxTreeChilds[child_index]->getType();
}

std::shared_ptr<ASTNode> ASTNode::child(unsigned int child_index)
{
	return getChild(child_index);
}

std::shared_ptr<ASTNode> ASTNode::getChild(unsigned int child_index)
{
	if(abstractSyntaxTreeChilds.size() <= child_index)
		std::cout << "Warning: ASTNode::getChild(unsigned int) got index " << child_index << " which is larger then the number of childs: " <<  abstractSyntaxTreeChilds.size() << std::endl;
	assert(abstractSyntaxTreeChilds.size() > child_index );
	return abstractSyntaxTreeChilds[child_index];
}

std::shared_ptr<ASTNode> ASTNode::getSoleChild()
{
	if(abstractSyntaxTreeChilds.size() != 1)
		std::cout << "Warning: ASTNode::getSoleChild() expected 1 child but found " << abstractSyntaxTreeChilds.size()<< std::endl;
	assert(abstractSyntaxTreeChilds.size() == 1);
	return abstractSyntaxTreeChilds[0];
}

std::shared_ptr<ASTNode> ASTNode::getSoleChild(std::string ofType)
{
	auto children = getChildren(ofType);
	if(children.size() != 1)
		std::cout << "Warning: ASTNode::getSoleChild expected 1 child but found " << children.size() << " children of type " << ofType << std::endl;
	assert(children.size() == 1);
	return children[0];
}

std::shared_ptr<ASTNode> ASTNode::getSoleDecendent(std::string ofType)
{
	auto Decendents = getDecendents(ofType);
	if(Decendents.size() != 1)
		std::cout << "Warning: ASTNode::getSoleDecendent expected 1 decendent but found " << Decendents.size() << " of type " << ofType << std::endl;
	assert(Decendents.size() == 1);
	return Decendents[0];
}

std::vector<std::shared_ptr<ASTNode> > ASTNode::getChildren(std::string ofType)
{
	std::vector<std::shared_ptr<ASTNode> > result;
	for(auto child : abstractSyntaxTreeChilds)
		if(child != nullptr)
			if(child->type == ofType)
				result.push_back(child);
	return result;

}

std::vector<std::shared_ptr<ASTNode> > ASTNode::getDecendents(std::string ofType)
{
	std::vector<std::shared_ptr<ASTNode> > result = getChildren(ofType);
	for(auto child : abstractSyntaxTreeChilds)
		if(child != nullptr)
		{
			auto subresult = child->getDecendents(ofType);
			result.insert(result.end(), subresult.begin(), subresult.end());
		}
	return result;
}





std::shared_ptr<ASTNode> ASTNode::getSoleChildWithContent(std::string payload)
{
	auto children = getChildrenWithContent(payload);
	if(children.size() != 1)
		std::cout << "Warning: ASTNode::getSoleChildWithContent expected 1 child but found " << children.size() << " with payload " << payload << std::endl;
	assert(children.size() == 1);
	return children[0];
}

std::shared_ptr<ASTNode> ASTNode::getSoleDecendentWithContent(std::string payload)
{
	auto Decendents = getDecendentsWithContent(payload);
	if(Decendents.size() != 1)
		std::cout << "Warning: ASTNode::getSoleDecendent expected 1 decendent but found " << Decendents.size() << " with payload " << payload << std::endl;
	assert(Decendents.size() == 1);
	return Decendents[0];
}

std::vector<std::shared_ptr<ASTNode> > ASTNode::getChildrenWithContent(std::string payload)
{
	std::vector<std::shared_ptr<ASTNode> > result;
	for(auto child : abstractSyntaxTreeChilds)
		if(child != nullptr)
			if(child->getContent() == payload)
				result.push_back(child);
	return result;

}

std::vector<std::shared_ptr<ASTNode> > ASTNode::getDecendentsWithContent(std::string payload)
{
	std::vector<std::shared_ptr<ASTNode> > result = getChildrenWithContent(payload);
	for(auto child : abstractSyntaxTreeChilds)
		if(child != nullptr)
		{
			auto subresult = child->getDecendentsWithContent(payload);
			result.insert(result.end(), subresult.begin(), subresult.end());
		}
	return result;
}

void ASTNode::eraseChild(unsigned int child_index)
{
	abstractSyntaxTreeChilds.erase(abstractSyntaxTreeChilds.begin()+child_index);
}

void ASTNode::replaceChild(std::shared_ptr<ASTNode> from, std::shared_ptr<ASTNode> to)
{
	int index = -1;
	for(unsigned int i=0; i < abstractSyntaxTreeChilds.size(); i++)
		if(from == abstractSyntaxTreeChilds[i])
		{
			index = i;
			break;
		}

	assert(index != -1 && "Warning: ASTNode::replaceChild did not find have child to be replaced in its children");
	abstractSyntaxTreeChilds[index] = to;
	to->abstractSyntaxTreeParent = from->abstractSyntaxTreeParent;
	from->abstractSyntaxTreeParent.reset();
}




} /* namespace abstract_syntax_tree */
} /* namespace edsl */
