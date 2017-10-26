/*
 * State.hpp
 *
 *  Created on: Mar 30, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_STATE_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_STATE_HPP_

#include <string>
#include <vector>
#include <rw/math.hpp>
#include <debug/Debug.hpp>
#include <color/colors.hpp>

namespace dsl
{

class State
{

public:
	State();
//	State(const State& s);
	virtual ~State();


	rw::math::Q	getQ() const;
	std::string getText() const;
	int getIO(int pin) const;
	int getGripper() const;

	void update(rw::math::Q q);
	void update(std::string text);
	void update(int pin, int value);
	void updateGripper(int q);


private:
	int _qGripper;
	rw::math::Q _q;
	std::string _str;
	std::vector<int> _ioports;
	std::vector<int> _iovalues;

	dsl::debug::Debug coutS;

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONTROLLER_STATE_HPP_ */
