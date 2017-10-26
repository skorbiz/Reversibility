/*
 * IOManipulation.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: josl
 */

#ifndef IOMANIPULATION_HPP_
#define IOMANIPULATION_HPP_

#include <DigitalIOGeneralInterfaceProxy.hpp>
#include <../src/model/basemodel/argument/quantities/Switch.hpp>
#include <../src/model/basemodel/command/CommandExecutable.hpp>

namespace dsl
{

class IOManipulation : public dsl::CommandExecutable
{

public:
	IOManipulation(std::shared_ptr<DigitalIOGeneralInterfaceProxy> DGIP, std::vector<int> ioports, dsl::switch_t::switch_t type);
	virtual ~IOManipulation();
	void execute();
	void executeBackwards();

	std::string getType();
	std::string getArgumentForward() const;
	std::string getArgumentBackward() const;

	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);

	void stateUpdate(int index,dsl::State & state);
	void stateUpdateSwapped(int index, dsl::State & state);


private:
	void execute(std::vector<uint8_t>& vec);

private:
	std::shared_ptr<DigitalIOGeneralInterfaceProxy> _DGIP;
	dsl::switch_t::switch_t _type;

	std::vector<int8_t> _ioports;
	std::vector<uint8_t> _valuesForward;
	std::vector<uint8_t> _valuesBackwards;

	bool _hadRedundantIOCommands;
	bool _hadRedundantIOCommandsSwappedTemp;

};

} /* namespace dsl */

#endif /* IOMANIPULATION_HPP_ */
