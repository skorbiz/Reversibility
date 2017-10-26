/*
 * RosAutoSpinner.h
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_ROSAUTOSPINNER_H_
#define REVERSIBLE_ASSEMBLY_SRC_ROSAUTOSPINNER_H_

#include <ros/ros.h>
#include <boost/thread.hpp>


class RosAutoSpinner
{

public:
	RosAutoSpinner(int argc, char **argv, std::string nodeName);
	virtual ~RosAutoSpinner();

private:
	void threadFunc();

	bool _runSpinThread;
	boost::thread _spinThread;
};

#endif /* REVERSIBLE_ASSEMBLY_SRC_ROSAUTOSPINNER_H_ */


