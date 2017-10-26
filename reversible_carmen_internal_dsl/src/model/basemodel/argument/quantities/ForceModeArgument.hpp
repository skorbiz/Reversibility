/*
 * ForceModeArgument.hpp
 *
 *  Created on: Jan 20, 2015
 *      Author: josl
 */

#ifndef FORCEMODEARGUMENT_HPP_
#define FORCEMODEARGUMENT_HPP_

#include <iostream>
#include <boost/array.hpp>
#include <../src/model/basemodel/argument/Argument.hpp>


namespace dsl
{

class ForceModeArgument : public Argument
{

public:
	enum axis_t{X = 0, Y = 1, Z = 2, RX = 3, RY = 4, RZ = 5};
	enum forcemodeType_t{POINT, FRAME, MOTION};

	ForceModeArgument();
	ForceModeArgument(forcemodeType_t type, boost::array<double,6> taskframe, boost::array<double,6> wrench, boost::array<double,6> limit, boost::array<bool,6> selectionvector);
	virtual ~ForceModeArgument();

	void setType(forcemodeType_t type);
	void setSelectionVector(axis_t axis);
	void setTaskFrame(axis_t axis, double value);
	void setWrench(axis_t axis, double value);
	void setLimit(axis_t axis, double value);

	int getType();
	boost::array<double,6> getTaskFrame();
	boost::array<bool,6> getSelectionVector();
	boost::array<double,6> getWrench();
	boost::array<double,6> getLimits();
	bool getActivationSignal();

	static ForceModeArgument getDeativationArgument();
	static ForceModeArgument getPushForward(double N);
	static ForceModeArgument getPushRight(double N);
	static ForceModeArgument getPushUp(double N);
	static ForceModeArgument getArgument(std::string id);

	friend std::ostream& operator<<(std::ostream& os, const ForceModeArgument& fma);

protected:
 	virtual void print(std::ostream& os) const;

private:
	forcemodeType_t _type;
	boost::array<double,6> _taskframe;
	boost::array<double,6> _wrench;
	boost::array<double,6> _limit;
	boost::array<bool,6> _selectionvector;
	bool _activation;

};

} /* namespace dsl */

#endif /* FORCEMODEARGUMENT_HPP_ */
