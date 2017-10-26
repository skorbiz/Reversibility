/*
 * ForceStatistics.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: josl
 */

#include "ForceStatistics.hpp"

#include <math.h>
#include <assert.h>
#include <iostream>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.hpp>
#include <iomanip>

namespace dsl {
namespace intelligentmove {

const double ForceStatistics::_defaultStdDiviationCurrentNorm = 0.20;
const double ForceStatistics::_defaultStdDiviationForceNorm = 0.20;

ForceStatistics::ForceStatistics( std::vector<Record> datasets ) :
		_datasets(datasets),
		_searchWindowSize(40),
		_useDefaultVraince(false),
		_samplesForce(_datasets.size()),
		_meanForce(6, 0,0),
		_medianForce(6, 0.0),
		_maxForce(6, 0.0),
		_minForce(6, 0.0),
		_varianceForce(6, 0.0),
		_stdDiviationForce(6, 0.0),
		_samplesCurrent(_datasets.size()),
		_meanCurrent(6, 0,0),
		_medianCurrent(6, 0.0),
		_maxCurrent(6, 0.0),
		_minCurrent(6, 0.0),
		_varianceCurrent(6, 0.0),
		_stdDiviationCurrent(6, 0.0)
{
	reset();
}

ForceStatistics::~ForceStatistics()
{
}

unsigned int ForceStatistics::sizeDataset()
{
	return _datasets.size();
}


void ForceStatistics::reset()
{
	_correspondingIndexes.clear();
	for(unsigned int i = 0; i < _datasets.size(); i++)
		_correspondingIndexes.push_back(0);
}

void ForceStatistics::update(rw::math::Q qCurrent)
{
	_qCurrent = qCurrent;
	_correspondingIndexes = calcCorrespondingIndexes();

	updateForce();
	updateCurrent();
	updateAdditionalIndexInfo();

	if(_useDefaultVraince)
	{
		_stdDiviationCurrent = rw::math::Q(6, _defaultStdDiviationCurrentNorm/sqrt(6));
		_stdDiviationForce = rw::math::Q(6, _defaultStdDiviationForceNorm/sqrt(6));
		_varianceCurrent = rw::math::Q(6, pow(_defaultStdDiviationForceNorm/sqrt(6),2));
		_varianceForce = rw::math::Q(6, pow(_defaultStdDiviationForceNorm/sqrt(6),2));
	}
}

void ForceStatistics::useDefaultVariance(bool value)
{
	_useDefaultVraince = value;
}


// **********************************
// *** Diviations from a force measure

rw::math::Q ForceStatistics::getMeanForceDiviation(const rw::math::Q & force)
{
	assert(force.size() == 6);
	assert(_meanForce.size() == 6);
	assert(_stdDiviationForce.size() == 6);

	rw::math::Q meanForceDiv(6);
	for(unsigned int i = 0; i < 6; i++)
		meanForceDiv[i] = (force[i]- _meanForce[i])/_stdDiviationForce[i];
	return meanForceDiv;
}

rw::math::Q ForceStatistics::getMaxForceDiviation(const rw::math::Q & force)
{
	assert(force.size() == 6);
	assert(_maxForce.size() == 6);
	assert(_minForce.size() == 6);

	rw::math::Q maxForceDiv(6);
	for(unsigned int i = 0; i < 6; i++)
		maxForceDiv[i] = (force[i]-_maxForce[i])/(_maxForce[i]-_minForce[i]);
	return maxForceDiv;
}

rw::math::Q ForceStatistics::getMinForceDiviation(const rw::math::Q & force)
{
	assert(force.size() == 6);
	assert(_maxForce.size() == 6);
	assert(_minForce.size() == 6);

	rw::math::Q minForceDiv(6);
	for(unsigned int i = 0; i < 6; i++)
		minForceDiv[i] = (_minForce[i]-force[i])/(_maxForce[i]-_minForce[i]);
	return minForceDiv;
}

std::vector<double> ForceStatistics::getDependentDiviation(const rw::math::Q & meanDiviations)
{
	assert(meanDiviations.size() == 6);

	std::vector<double> div;
	for(unsigned int i = 0; i < 6; i++)
		div.push_back(std::abs(meanDiviations[i]));

	std::sort(div.begin(),div.end(), [](const double& lhs, const double& rhs){return lhs > rhs;});

	double value = 0;
	std::vector<double> ra;
	for(unsigned int i = 0; i < div.size(); i++)
	{
		value += div[i];
		ra.push_back(value/(i+1));
	}

	return ra;
}


// **********************************
// *** Calculations on the static datasets

void ForceStatistics::updateAdditionalIndexInfo()
{
	assert(_datasets.size() == _correspondingIndexes.size());

	std::vector<double> completionPercentage;
	for(unsigned int d = 0; d < _datasets.size(); d++)
	{
		unsigned int iMax = _datasets[d].qSample.size();
		double percentage = ((double) _correspondingIndexes[d])/ ((double) iMax);
		completionPercentage.push_back(percentage);
	}
	_corrIndexPercantage = completionPercentage;


	std::vector<double> qDistance;
	for(unsigned int d = 0; d < _datasets.size(); d++)
	{
		rw::math::Q qMax = _datasets[d].qSample.back();
		rw::math::Q qC =_datasets[d].qSample[_correspondingIndexes[d]];
		double percentage = (qMax-qC).norm2();
		qDistance.push_back(percentage);
	}
	_corrIndexQDistance= qDistance;
}


void ForceStatistics::updateForce()
{
	assert(_datasets.size() == _samplesForce.size());

	for(unsigned int d = 0; d < _datasets.size(); d++)
		_samplesForce[d] = _datasets[d].forceSample[ _correspondingIndexes[d] ];

	std::vector<double> inputs(_datasets.size());

	for(int qi = 0; qi < 6; qi++)
	{
		for(unsigned int d = 0; d < _datasets.size(); d++)
			inputs[d] = _samplesForce[d][qi];

		_meanForce[qi] = Statistics::calcMean(inputs);

		double variance = Statistics::calcVariance(inputs);
		_stdDiviationForce[qi] = std::sqrt(variance);
		_varianceForce[qi] = variance;

		if(inputs.size() == 0)
			inputs.push_back(NAN);

		std::sort(inputs.begin(),inputs.end());
		_medianForce[qi] = inputs[inputs.size()/2];
		_maxForce[qi] = inputs[inputs.size()-1];
		_minForce[qi] = inputs[0];
	}
}


void ForceStatistics::updateCurrent()
{
	assert(_datasets.size() == _samplesForce.size());

	for(unsigned int d = 0; d < _datasets.size(); d++)
		_samplesCurrent[d] = _datasets[d].iActual[ _correspondingIndexes[d] ];

	std::vector<double> inputs(_datasets.size());

	for(int qi = 0; qi < 6; qi++)
	{
		for(unsigned int d = 0; d < _datasets.size(); d++)
			inputs[d] = _samplesCurrent[d][qi];

		_meanCurrent[qi] = Statistics::calcMean(inputs);

		double variance = Statistics::calcVariance(inputs);
		_stdDiviationCurrent[qi] = std::sqrt(variance);
		_varianceCurrent[qi] = variance;

		if(inputs.size() == 0)
			inputs.push_back(NAN);

		std::sort(inputs.begin(),inputs.end());
		_medianCurrent[qi] = inputs[inputs.size()/2];
		_maxCurrent[qi] = inputs[inputs.size()-1];
		_minCurrent[qi] = inputs[0];
	}
}


// **********************************
// *** Data extraction

std::vector<unsigned int> ForceStatistics::calcCorrespondingIndexes() const
{
	std::vector<unsigned int> indexes;
	for(unsigned int i = 0; i < _datasets.size(); i++)
		indexes.push_back( calcCorrespondingIndex(_qCurrent, _correspondingIndexes[i], _datasets[i]) );
	return indexes;
}

unsigned int ForceStatistics::calcCorrespondingIndex(const rw::math::Q & qCurrent, unsigned int indexStart, const Record & recordedData) const
{
	assert(recordedData.qSample.size() == recordedData.forceSample.size());
	assert(recordedData.qSample.size() == recordedData.iActual.size());

	unsigned int indexEnd = indexStart + _searchWindowSize;
	if(indexEnd > recordedData.qSample.size())
		indexEnd = recordedData.qSample.size();

	unsigned int bestIndex = 0;
	double bestDistance = std::numeric_limits<double>::infinity();

//	for(unsigned int i = 0; i < recordedData.qSample.size(); i++)
	for(unsigned int i = indexStart; i < indexEnd; i++)
	{
		double distance = (qCurrent-recordedData.qSample[i]).norm2();

		if(distance < bestDistance)
		{
			bestDistance = distance;
			bestIndex = i;
		}
	}

	return bestIndex;
}

// **********************************
// *** Basic statistic calculations



} /* namespace intelligentmove */
} /* namespace dsl */
