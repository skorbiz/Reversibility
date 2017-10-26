/*
 * Statistics.hpp
 *
 *  Created on: Mar 7, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_STATISTICS_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_STATISTICS_HPP_

#include <vector>
#include <rw/math/Q.hpp>

namespace dsl {
namespace intelligentmove {

class Statistics {
public:
	Statistics();
	virtual ~Statistics();

	struct minMaxMedian
	{
		double min;
		double max;
		double median;
	};

	static std::vector<rw::math::Q> getColumn(const std::vector<std::vector<rw::math::Q> >& vec, int col);
	static std::vector<double> getColumn(const std::vector<std::vector<double> > & vec, int col);
	static std::vector<double> getColumn(const std::vector<rw::math::Q> & vec, int col);

	static std::vector<double> calcNorms(const std::vector<rw::math::Q> & force);
	static double calcNorm(const rw::math::Q & force);

	static std::vector<rw::math::Q> calcDQ(const std::vector<rw::math::Q> & force);
	static std::vector<rw::math::Q> calcRunningAvarge(const std::vector<rw::math::Q> & force, int length, bool Weighed);
	static std::vector<rw::math::Q> calcRunningAvarge(const std::vector<rw::math::Q> & force, int length);
	static std::vector<rw::math::Q> calcRunningAvargeWeighed(const std::vector<rw::math::Q> & force, int length);

	static rw::math::Q calcBackwardsAccumulation(const std::vector<rw::math::Q> & input, unsigned int length);
	static rw::math::Q calcBcWeighedAccumulation(const std::vector<rw::math::Q> & input, unsigned int length);
	static rw::math::Q calcBackwardsAccumulation(const std::vector<rw::math::Q> & input, unsigned int length, std::vector<rw::math::Q>::const_iterator iteratorEnd);
	static rw::math::Q calcBcWeighedAccumulation(const std::vector<rw::math::Q> & input, unsigned int length, std::vector<rw::math::Q>::const_iterator iteratorEnd);

	static std::vector<double> calcRelative(const std::vector<double> & input);
	static std::vector<double> getSampleNumbers(unsigned int inputSize);


	static double calcMean(const std::vector<double> & input);
	static double calcVariance(const std::vector<double> & input);
	static double calcMax(const std::vector<double> & input);
	static double calcMin(const std::vector<double> & input);

	static double calcSum(const rw::math::Q & input);

};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_STATISTICS_HPP_ */
