/*
 * Variable.h
 *
 *  Created on: Jun 20, 2017
 *      Author: josl
 */

#include <memory>
#include <string>

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_VARIABLE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_VARIABLE_H_

namespace model {

template< class ClassType>
class Variable
{
public:
	Variable();
	Variable(std::string name, ClassType variable);
	virtual ~Variable();

	void set(ClassType obj);
	ClassType get();
	ClassType& getRef();

	std::string name;
	std::shared_ptr<std::shared_ptr<ClassType> > variable;
};



// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************

template <class ClassType>
Variable<ClassType>::Variable()
{
}

template <class ClassType>
Variable<ClassType>::Variable(std::string name, ClassType aVar) : name(name)
{
	variable = std::make_shared<std::shared_ptr<ClassType> >(std::make_shared<ClassType>(aVar));
}


template <class ClassType>
Variable<ClassType>::~Variable()
{
}


template <class ClassType>
void Variable<ClassType>::set(ClassType obj)
{
	std::shared_ptr<ClassType> ptr = std::make_shared<ClassType>(obj);
	(variable.get())->swap(ptr);

//	std::shared_ptr<model::ControlUnit> a;
//	std::shared_ptr<model::ControlUnit> c;
//	std::shared_ptr<std::shared_ptr<model::ControlUnit> >b;
//	b->get() = c;


}

template <class ClassType>
ClassType Variable<ClassType>::get()
{
	return *(variable->get());
}

template <class ClassType>
ClassType& Variable<ClassType>::getRef()
{
	return *(variable->get());
}




//
//template< class T2 >
//std::ostream& operator<<(std::ostream& os, const MemoryModel<T2>& model)
//{
//
//	os << "Memory contains number of elements: " << model.memory.size() << std::endl;
//	typename std::map<std::string, T2>::const_iterator iter;
//	for (iter = model.memory.begin(); iter != model.memory.end(); iter++)
//		os << "Key: " << iter->first << std::endl;
//	return os;
//}

} /* namespace model */
#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_VARIABLE_H_ */
