/*
 * IOPorts.hpp
 *
 *  Created on: Nov 12, 2014
 *      Author: josl
 */

#ifndef IOPORTS_HPP_
#define IOPORTS_HPP_

#include <vector>
#include <initializer_list>
#include <../src/model/basemodel/argument/Argument.hpp>
#include <../src/model/basemodel/argument/Datatype.hpp>

namespace dsl
{

class IOPorts : public Argument, public Datatype<std::vector<int> >
{

public:
	IOPorts();
	IOPorts(int ioport);
	IOPorts(std::initializer_list<int> ioports);
	virtual ~IOPorts();

	template<typename T, typename... Args>
	IOPorts(T t, Args... args);

	template<typename T, typename... Args>
	void append(T t, Args... args);

	template <typename T>
	void append(T t);

	void append(int t);

	friend std::ostream& operator<<(std::ostream& os, const IOPorts& i);

};

//
// TEMPLATEED FUNCTIONS
//

template<typename T, typename... Args>
IOPorts::IOPorts(T t, Args... args)
{
	append(t);
	append(args...);
}

template<typename T, typename... Args>
void IOPorts::append(T t, Args... args)
{
	append(t);
	append(args...);
}

template <typename T>
void IOPorts::append(T t)
{
	std::cerr << "Only Integers can be IO ports" << std::endl;
}


} /* namespace dsl */

#endif /* IOPORTS_HPP_ */
