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


template< class ClassType>
class MemoryModel
{

public:
	MemoryModel(std::string type);
	virtual ~MemoryModel();

	void save(std::string name, ClassType obj);
	void erase(std::string name);
	ClassType& getRef(std::string name);
	bool doContain(std::string name);


	int getSize();

	template <class T2>
	friend std::ostream& operator<<(std::ostream& os, const MemoryModel<T2> & model);

private:
	std::map<std::string, ClassType> memory;
	std::string type;

};

// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************

template <class ClassType>
MemoryModel<ClassType>::MemoryModel(std::string type) : type(type)
{
}


template <class ClassType>
MemoryModel<ClassType>::~MemoryModel()
{
}


template <class ClassType>
void MemoryModel<ClassType>::save(std::string name, ClassType obj)
{
	if(memory.count(name)){
		std::cerr << "A object of type "<< type << " with name \"" << name << "\" already exist." << std::endl;
		exit(-1);
	}

	memory[name] = obj;
}


template <class ClassType>
void MemoryModel<ClassType>::erase(std::string name)
{
	if(!memory.count(name)){
		std::cerr << "No object of type "<< type << " with name \"" << name << "\" exist." << std::endl;
		exit(-1);
	}
	memory.erase(name);
}



template <class ClassType>
ClassType& MemoryModel<ClassType>::getRef(std::string name)
{
	if(memory.count(name))
		return memory[name];

	std::cerr << "No object of type "<< type << " with name \"" << name << "\" exist." << std::endl;
	std::cout << *this << std::endl;
	exit(-1);
}

template <class ClassType>
bool MemoryModel<ClassType>::doContain(std::string name)
{
	if(memory.count(name))
		return true;
	return false;
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



#endif /* MEMORYMODEL_H_ */
