/*
 * MemoryModel.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#ifndef MEMORYMODEL_HPP_
#define MEMORYMODEL_HPP_

#include <iostream>
#include <map>
#include <../src/model/basemodel/argument/Argument.hpp>

namespace dsl
{

template< class ClassType>
class MemoryModel
{

public:
	MemoryModel();
	virtual ~MemoryModel();

	void saveObject(std::string name, ClassType obj);
	ClassType getObject(std::string name);

	int getSize();

//	void newObject();
//	void getObject();
//	void getCurrentObject();
//	void saveCurrentObject();


	template <class T2>
	friend std::ostream& operator<<(std::ostream& os, const MemoryModel<T2> & model);

private:
//	std::string currentName;
//	ClassType currentObj;

	std::map<std::string, ClassType> memory;

};

// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************

template <class ClassType>
MemoryModel<ClassType>::MemoryModel()
{
}


template <class ClassType>
MemoryModel<ClassType>::~MemoryModel()
{
}


template <class ClassType>
void MemoryModel<ClassType>::saveObject(std::string name, ClassType obj)
{
	if(memory.count(name)){
		std::cerr << "A object with name \"" << name << "\" already exist." << std::endl;
		exit(-1);
	}

	memory[name] = obj;
}



template <class ClassType>
ClassType MemoryModel<ClassType>::getObject(std::string name)
{
	if(memory.count(name))
		return memory[name];

	std::cerr << "No object with name \"" << name << "\" exist." << std::endl;
	exit(-1);
}

template <class ClassType>
int MemoryModel<ClassType>::getSize()
{
	return memory.size();
}



template< class T2 >
std::ostream& operator<<(std::ostream& os, const MemoryModel<T2>& model)
{

	os << "Memory contains number of elements: " << model.memory.size() << std::endl;
	typename std::map<std::string, T2>::const_iterator iter;
	for (iter = model.memory.begin(); iter != model.memory.end(); iter++)
		os << "Key: " << iter->first << std::endl;
	return os;
}



} /* namespace dsl */

#endif /* MEMORYMODEL_H_ */
