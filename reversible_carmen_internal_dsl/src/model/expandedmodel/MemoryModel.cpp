/*
 * MemoryModel.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#include "MemoryModel.hpp"

namespace dsl
{

template<>
std::ostream& operator<<(std::ostream& os, const MemoryModel<dsl::Argument*>& model)
{
	os << "Memory print from template specialization 0" << std::endl;
	os << "Memory contains number of elements: " << model.memory.size() << std::endl;
	typename std::map<std::string, dsl::Argument*>::const_iterator iter;
	for (iter = model.memory.begin(); iter != model.memory.end(); iter++)
	{
		os << "Key: " << iter->first << "\t";
		os << "value: " <<*(iter->second) << std::endl;
	}
	return os;
}

//template<>
//std::ostream& operator<<(std::ostream& os, const MemoryModel<dsl::Argument&>& model)
//{
//	os << "Memory print from template specialization 1" << std::endl;
//	os << "Memory contains number of elements: " << model.memory.size() << std::endl;
//	typename std::map<std::string, dsl::Argument>::const_iterator iter;
//	for (iter = model.memory.begin(); iter != model.memory.end(); iter++)
//	{
//		os << "Key: " << iter->first << "\t";
//		os << "value: " <<iter->second << std::endl;
//	}
//	return os;
//}

template<>
std::ostream& operator<<(std::ostream& os, const MemoryModel<dsl::Argument>& model)
{
	os << "Memory print from template specialization 2" << std::endl;
	os << "Memory contains number of elements: " << model.memory.size() << std::endl;
	typename std::map<std::string, dsl::Argument>::const_iterator iter;
	for (iter = model.memory.begin(); iter != model.memory.end(); iter++)
	{
		os << "Key: " << iter->first << "\t";
		os << "value: " <<iter->second << std::endl;
	}
	return os;
}

} /* namespace dsl */
