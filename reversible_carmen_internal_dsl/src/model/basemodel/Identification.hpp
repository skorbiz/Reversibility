/*
 * Identification.hpp
 *
 *  Created on: Mar 12, 2015
 *      Author: josl
 */

#ifndef IDENTIFICATION_HPP_
#define IDENTIFICATION_HPP_

#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>

namespace dsl
{

class Identification
{

public:
	typedef std::shared_ptr<Identification> Ptr;
//	typedef boost::shared_ptr<Identification> Ptr;

	Identification(std::string sequence, int instance, std::string number, int depth);
	Identification(std::string sequence, int instance, int number, int depth);
	virtual ~Identification();

	std::string getOrigin() const;
	std::string getInstance() const;
	std::string getGrouping() const;
	std::string getNumber() const;
	int			getDepth() const;

	std::string toString() const;
	operator std::string() const;
	friend std::ostream& operator<<(std::ostream& os, const Identification& id);


private:
	std::string _origin;
	std::string _instance;
	std::string _number;
	int	_depth;

};

} /* namespace dsl */

#endif /* IDENTIFICATION_HPP_ */
