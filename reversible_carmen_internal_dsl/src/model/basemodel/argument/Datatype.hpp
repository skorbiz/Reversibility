/*
 * Datatype.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: josl
 */

#ifndef DATATYPE_HPP_
#define DATATYPE_HPP_

#include <iostream>

namespace dsl
{

template <class T = double>
class Datatype
{

public:
	Datatype();
	Datatype(T value);
	virtual ~Datatype();

	virtual void set(T value);
	virtual T& get();
	const virtual T& get() const;

	template< class T2>
	friend std::ostream& operator<<(std::ostream& os, const Datatype<T2>& d);

private:
	T _value;

};


// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************

template <class T>
Datatype<T>::~Datatype()
{
}

template <class T>
Datatype<T>::Datatype()
{
}

template <class T>
Datatype<T>::Datatype(T value)
{
	_value = value;
}

template <class T>
void Datatype<T>::set(T value)
{
	_value = value;
}

template <class T>
T & Datatype<T>::get()
{
	return _value;
}

template <class T>
const T& Datatype<T>::get() const
{
	return _value;
}

template< class T2 >
std::ostream& operator<<(std::ostream& os, const Datatype<T2>& l)
{
    os << l._value;
    return os;
}

} /* namespace dsl */

#endif /* DATATYPE_HPP_ */




