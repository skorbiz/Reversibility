/*
 * Switch.hpp
 *
 *  Created on: Nov 12, 2014
 *      Author: josl
 */

#ifndef SWITCH_HPP_
#define SWITCH_HPP_

#include <../src/model/basemodel/argument/Argument.hpp>
#include <../src/model/basemodel/argument/Datatype.hpp>

namespace dsl
{

namespace switch_t
{
	enum class switch_t {ON, OFF, TOGGLE, UNSPECIFIED};
}

class Switch : public Argument, public Datatype<switch_t::switch_t>
{

public:
	static const enum switch_t::switch_t on = switch_t::switch_t::ON;
	static const enum switch_t::switch_t off = switch_t::switch_t::OFF;
	static const enum switch_t::switch_t toggle = switch_t::switch_t::TOGGLE;

public:
	Switch();
	Switch(switch_t::switch_t value);
	virtual ~Switch();

	friend std::ostream& operator<<(std::ostream& os, const Switch& s);


};

} /* namespace dsl */

#endif /* SWITCH_HPP_ */
