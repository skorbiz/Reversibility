/*
 * Grasp.hpp
 *
 *  Created on: Jan 20, 2015
 *      Author: josl
 */

#ifndef GRASP_HPP_
#define GRASP_HPP_

#include <RobotiqInterfaceProxy.hpp>
#include <../src/model/basemodel/command/CommandExecutable.hpp>


namespace dsl
{

class Grasp : public dsl::CommandExecutable
{
public:
	Grasp(std::shared_ptr<RobotiqInterfaceProxy> RIQP, int q, int speed, int force);
	virtual ~Grasp();

	void execute();
	void executeBackwards();
	std::string getType();

	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);

	std::string getGraspStatus(int q) const;
	std::string getArgumentForward() const;
	std::string getArgumentBackward() const;


private:
	std::shared_ptr<RobotiqInterfaceProxy> _RIQP;
	int _qTo;
	int _qFrom;
	int _speed;
	int _force;

};

} /* namespace dsl */

#endif /* GRASP_HPP_ */

