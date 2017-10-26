/*
 * Statistics.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: josl
 */

#include "Statistics.hpp"
#include <assert.h>

namespace dsl {
namespace intelligentmove {

Statistics::Statistics()
{
}

Statistics::~Statistics()
{
}

std::vector<rw::math::Q> Statistics::getColumn(const std::vector<std::vector<rw::math::Q> >& vec, int col)
{
	assert(vec.size() >= col);
	std::vector<rw::math::Q> res;
	for(std::vector<rw::math::Q> q : vec)
		res.push_back(q[col]);
	return res;
}

std::vector<double> Statistics::getColumn(const std::vector<std::vector<double> > & vec, int col)
{
	assert(vec.size() >= col);
	std::vector<double> res;
	for(std::vector<double> q: vec)
		res.push_back(q[col]);
	return res;
}

std::vector<double> Statistics::getColumn(const std::vector<rw::math::Q> & vec, int col)
{
	assert(vec.size() >= col);
	std::vector<double> res;
	for(rw::math::Q q : vec)
		res.push_back(q[col]);
	return res;
}

std::vector<rw::math::Q> Statistics::calcDQ(const std::vector<rw::math::Q> & force)
{
	assert(force.size() > 2);
	std::vector<rw::math::Q> res;
	res.push_back(force[0]);
	for(unsigned int i = 1; i < force.size(); i++)
		res.push_back(force[i]-force[i-1]);
	return res;
}

std::vector<rw::math::Q> Statistics::calcRunningAvarge(const std::vector<rw::math::Q> & force, int length, bool weighed)
{
	if(weighed)
		return calcRunningAvargeWeighed(force, length);
	return calcRunningAvarge(force, length);
}

std::vector<rw::math::Q> Statistics::calcRunningAvarge(const std::vector<rw::math::Q> & force, int length)
{
	std::vector<rw::math::Q> res;
	for(unsigned int i = 0; i < force.size(); i++)
		res.push_back( calcBcWeighedAccumulation(force, length, force.begin()+i+1)/length);
	return res;
}

std::vector<rw::math::Q> Statistics::calcRunningAvargeWeighed(const std::vector<rw::math::Q> & force, int length)
{
	std::vector<rw::math::Q> res;
	for(unsigned int i = 0; i < force.size(); i++)
		res.push_back( calcBcWeighedAccumulation(force, length, force.begin()+i+1)/length);
	return res;
}

rw::math::Q Statistics::calcBackwardsAccumulation(const std::vector<rw::math::Q> & input, unsigned int length)
{
	return calcBackwardsAccumulation(input, length, input.end());
}

rw::math::Q Statistics::calcBcWeighedAccumulation(const std::vector<rw::math::Q> & input, unsigned int length)
{
	return calcBcWeighedAccumulation(input, length, input.end());
}

rw::math::Q Statistics::calcBackwardsAccumulation(const std::vector<rw::math::Q> & input, unsigned int length, std::vector<rw::math::Q>::const_iterator iteratorEnd)
{
	assert(input.back().size() == 6);
	assert(((int)input.size()) > 0);
	assert(length > 0);

	auto start = iteratorEnd-length;

	if(std::distance(start, input.begin()) > 0)
		start = input.begin();

	rw::math::Q sum = std::accumulate(start, iteratorEnd, rw::math::Q(6,0.0));
	return sum;
}

rw::math::Q Statistics::calcBcWeighedAccumulation(const std::vector<rw::math::Q> & input, unsigned int length, std::vector<rw::math::Q>::const_iterator iteratorEnd)
{
	assert(input.back().size() == 6);
	assert(((int)input.size()) > -1);
	assert(length > 0);

	auto start = iteratorEnd-length;
	int offset = std::distance(start, input.begin());

	if(offset < 0)
		offset = 0;

	std::vector<double> w;
	for(unsigned int i = 0; i < length; i++)
		w.push_back(i+1);

	double totalW = std::accumulate(w.begin(),w.end(),0.0);

	rw::math::Q sum(6,0.0);
	for(unsigned int i = offset; i < length; i++)
	{
		rw::math::Q val = (*(start+i));
		double scale = (w[i]/totalW)*length;
		sum += val * scale;
	}
	return sum;
}

std::vector<double> Statistics::calcNorms(const std::vector<rw::math::Q> & force)
{
	std::vector<double> res;
	for(rw::math::Q f : force)
		res.push_back(calcNorm(f));
	return res;
}

double Statistics::calcNorm(const rw::math::Q & forceVector)
{
	double force = 0;
	force += forceVector(0)*forceVector(0);
	force += forceVector(1)*forceVector(1);
	force += forceVector(2)*forceVector(2);
	return std::sqrt(force);
}

std::vector<double> Statistics::calcRelative(const std::vector<double> & input)
{
	std::vector<double> res;
	for(double i : input)
		res.push_back(i-input.front());
	return res;
}

std::vector<double> Statistics::getSampleNumbers(unsigned int inputSize)
{
	std::vector<double> res;
	for(int i = 1; i <= inputSize; i++)
		res.push_back(i);
	return res;
}

double Statistics::calcMean(const std::vector<double> & input)
{
	double mean = 0;
	for(double f : input)
		mean += f;
	return mean/input.size();
}

double Statistics::calcVariance(const std::vector<double> & input)
{
	double mean = calcMean(input);
	double variance = 0;
	for(double f : input)
		variance += (f-mean)*(f-mean);
	return variance/input.size();
}

double Statistics::calcMax(const std::vector<double> & input)
{
	assert(input.size() > 0);
	double max = input[0];
	for(double f : input)
		if(max < f)
			max = f;
	return max;
}

double Statistics::calcMin(const std::vector<double> & input)
{
	assert(input.size() > 0);
	double min = input[0];
	for(double f : input)
		if(min > f)
			min = f;
	return min;
}

double Statistics::calcSum(const rw::math::Q & input)
{
	double sum = 0;
	for(unsigned int i = 0; i < input.size(); i++)
		sum+=input[i];
	return sum;
}

} /* namespace intelligentmove */
} /* namespace dsl */
