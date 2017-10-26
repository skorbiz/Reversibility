/*
 * Argument.hpps
 *
 *  Created on: Nov 1, 2014
 *      Author: josl
 */

#ifndef ARGUMENT_HPP_
#define ARGUMENT_HPP_

#include <iostream>

namespace dsl {

class Argument
{

public:
	Argument();
	virtual ~Argument();

protected:
    virtual void print(std::ostream& os) const;

public:
    friend inline std::ostream& operator<<(std::ostream& os, const Argument& data)
    {
    	data.print(os);
        return os;
    }


};


} /* namespace dsl */

#endif /* ARGUMENT_HPP_ */
