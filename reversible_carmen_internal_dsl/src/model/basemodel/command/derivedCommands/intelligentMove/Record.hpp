/*
 * Record.hpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORD_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORD_HPP_

#include <rw/math/Q.hpp>
#include <string>
#include <vector>

namespace dsl {
namespace intelligentmove {

class Record
{
public:
	enum MoveType {servo, move};

public:
	Record();
	virtual ~Record();

public:
	rw::math::Q qStart;
	rw::math::Q qEnd;

	std::vector<rw::math::Q> forceSample;
	std::vector<rw::math::Q> qSample;
	std::vector<rw::math::Q> tcpPoseSample;
	std::vector<rw::math::Q> iActual;
	std::vector<rw::math::Q> iControl;
	std::vector<double> safetyMode;
	std::vector<double> errorState;
	std::vector<double> timestamp;
	std::vector<double> timestampPC;

	double speed;
	double acceleration;

	MoveType type;

	bool isActive;
	bool successfullMove;

	std::string note;
};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORD_HPP_ */
