/*
 * Identification23.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_IDENTIFICATION23_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_IDENTIFICATION23_H_

#include <iostream>
#include <memory>
#include <string>

namespace model
{

class Identification23
{

public:
	typedef std::shared_ptr<Identification23> Ptr;

	Identification23(std::string sequence, int instance, std::string number, int depth);
	Identification23(std::string sequence, int instance, int number, int depth);
	virtual ~Identification23();

	std::string getOrigin() const;
	std::string getInstance() const;
	std::string getGrouping() const;
	std::string getNumber() const;
	int			getDepth() const;

	std::string toString() const;
	operator std::string() const;
	friend std::ostream& operator<<(std::ostream& os, const Identification23& id);


private:
	std::string _origin;
	std::string _instance;
	std::string _number;
	int	_depth;
};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_IDENTIFICATION23_H_ */
