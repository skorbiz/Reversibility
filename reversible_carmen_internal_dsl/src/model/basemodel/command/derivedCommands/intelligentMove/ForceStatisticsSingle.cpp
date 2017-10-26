/*
 * ForceStatisticsSingle.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: josl
 */

#include "ForceStatisticsSingle.hpp"

#include <math.h>
#include <assert.h>
#include <iostream>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.hpp>
#include <iomanip>
#include <color/colors.hpp>
#include <rw/math/RPY.hpp>

namespace dsl {
namespace intelligentmove {

ForceStatisticsSingle::ForceStatisticsSingle(std::vector<Record> datasets) :
				_meanCurrent(6, 0,0),
				_medianCurrent(6, 0.0),
				_maxCurrent(6, 0.0),
				_minCurrent(6, 0.0),
				_varianceCurrent(6, 0.0),
				_stdDiviationCurrent(6, 0.0),
				_corrIndexPercantage(0),
				_corrIndexQDistance(-1),
				_corrIndexBestMatch(0),
				_corrIndexLast(0),
				_corrIndexDirect(0),
				_windowSizeSearch(40),
				_windowSizeKernal(7),
				_offsetQ(6, 0.0),
				_offsetTcpMeasured(6, 0.0),
				_offsetTcpCalculated(6, 0.0)
{

	deb.setPrefix("ForceStatisticsSingle: ");
	deb.setColor(dsl::color::CYAN);
	deb.disable();

	if(datasets.size() == 1)
		_dataset = datasets.front();
	else
		std::cerr << dsl::color::red("ForceStaticsticsSingle did not contain the correct number of records") << std::endl;

	reset();
}

ForceStatisticsSingle::~ForceStatisticsSingle()
{
}


void ForceStatisticsSingle::reset()
{
	_correspondingIndex = 0;
	_samplesCurrent.clear();
	_meanCurrent =rw::math::Q(6, 0.0);
	_medianCurrent = rw::math::Q(6, 0.0);
	_maxCurrent = rw::math::Q(6, 0.0);
	_minCurrent = rw::math::Q(6, 0.0);
	_varianceCurrent = rw::math::Q(6, 0.0);
	_stdDiviationCurrent = rw::math::Q(6, 0.0);
	_corrIndexDirect = 0;
}

//void ForceStatisticsSingle::update(rw::math::Q qCurrent)
//{
//	_corrIndexLast = _correspondingIndex;
//	_correspondingIndex = calcCorrespondingIndex(qCurrent, _dataset.qSample, _correspondingIndex);
//
//	updateCurrent();
//
//	updateAdditionalIndexInfo();
//}
//
//void ForceStatisticsSingle::update(rw::math::Q tcpCurrent, rw::math::Transform3D<double> expectedOffset)
//{
//	rw::math::Q rectifyedTCP = calcOffsetPose(tcpCurrent, expectedOffset);
//
//	_corrIndexLast = _correspondingIndex;
//
//	_correspondingIndex = calcCorrespondingIndex(rectifyedTCP, _dataset.tcpPoseSample, _correspondingIndex);
//
//	_corrIndexDirect++;
//	updateCurrent();
//	updateAdditionalIndexInfo();
//}

void ForceStatisticsSingle::update(int stategy, rw::math::Q q, rw::math::Q p, rw::math::Q i)
{

	rw::math::Q rectifyedQ = q + _offsetQ;
	rw::math::Q rectifyedTcpMeasured = p + _offsetTcpMeasured;
	rw::math::Q rectifyedTcpCalculated = p + _offsetTcpCalculated;

	if(stategy == 0)
		_correspondingIndex = calcCorrespondingIndex(q, _dataset.qSample, _correspondingIndex, 40);
	else if(stategy == 1)
		_correspondingIndex = calcCorrespondingIndex(q, _dataset.qSample, _correspondingIndex, 5);
	else if(stategy == 2)
		_correspondingIndex = calcCorrespondingIndex(q, _dataset.qSample, _corrIndexDirect-10, 20);
	else if(stategy == 3)
		_correspondingIndex = calcCorrespondingIndex(rectifyedQ, _dataset.qSample, _correspondingIndex, 5);
	else if(stategy == 4)
		_correspondingIndex = calcCorrespondingIndex(rectifyedTcpMeasured, _dataset.tcpPoseSample, _correspondingIndex, 5);
	else if(stategy == 5)
		_correspondingIndex = calcCorrespondingIndex(rectifyedTcpCalculated, _dataset.tcpPoseSample, _correspondingIndex, 5);
	else if(stategy == 6)
		_correspondingIndex = calcCorrespondingIndex(p, _dataset.tcpPoseSample, _correspondingIndex, 5);
	else if(stategy == 7)
		_correspondingIndex = calcCorrespondingIndex(i, _dataset.iActual, _corrIndexDirect-10, 20);
	else
		assert(false && "Correct strategy not selected");

	_corrIndexDirect++;
	updateCurrent();
	updateAdditionalIndexInfo();

}


// **********************************
// *** Calculations on the static datasets

void ForceStatisticsSingle::updateAdditionalIndexInfo()
{
	unsigned int iMax = _dataset.qSample.size();
	double percentage = ((double) _correspondingIndex)/ ((double) iMax);
	_corrIndexPercantage = percentage;

	rw::math::Q qMax = _dataset.qSample.back();
	rw::math::Q qC =_dataset.qSample[_correspondingIndex];
	double percentage2 = (qMax-qC).norm2();
	_corrIndexQDistance= percentage2;

}




void ForceStatisticsSingle::updateCurrent()
{

	_samplesCurrent = calcSampleCurrents(_correspondingIndex, _dataset);

	std::vector<double> inputs(_samplesCurrent.size());

	for(int qi = 0; qi < 6; qi++)
	{
		for(unsigned int d = 0; d < _samplesCurrent.size(); d++)
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

std::vector<rw::math::Q> ForceStatisticsSingle::calcSampleCurrents(unsigned int index, const Record & recordedData) const
{
	assert(_windowSizeKernal % 2 == 1);
	assert(0 <= (int) recordedData.qSample.size());

	int leftIndex = index -(_windowSizeKernal-1)/2;
	int rightIndex = index +(_windowSizeKernal-1)/2;

	if(leftIndex < 0)
		leftIndex = 0;

	if(rightIndex >= (int) recordedData.qSample.size())
		rightIndex = recordedData.qSample.size() -1;

	std::vector<rw::math::Q> sampleCurrents(
			recordedData.iActual.begin()+leftIndex,
			recordedData.iActual.begin()+rightIndex+1);

	return sampleCurrents;
}

unsigned int ForceStatisticsSingle::calcCorrespondingIndex(const rw::math::Q & currentInput, const std::vector<rw::math::Q>& recordedSamples, unsigned int indexStart, unsigned int windowSize)
{
	unsigned int indexEnd = indexStart + windowSize;

	if((int) indexStart < 0)
		indexStart = 0;

	if(indexStart >= recordedSamples.size())
		indexStart = recordedSamples.size() -1;

	if(indexEnd > recordedSamples.size())
		indexEnd = recordedSamples.size();

	unsigned int bestIndex = 0;
	double bestDistance = std::numeric_limits<double>::infinity();

	for(unsigned int i = indexStart; i < indexEnd; i++)
	{
		double distance = (currentInput-recordedSamples[i]).norm2();

		if(distance < bestDistance)
		{
			bestDistance = distance;
			bestIndex = i;
		}
	}

	_corrIndexBestMatch = bestDistance;
	return bestIndex;
}

void ForceStatisticsSingle::calcOffsets(const rw::math::Q & q, const rw::math::Q & p, const rw::math::Transform3D<> & offsetTcpExpected)
{
	assert(q.size() == 6);
	assert(p.size() == 6);

	_offsetQ = _dataset.qSample[_correspondingIndex]-q;
	_offsetTcpMeasured = _dataset.tcpPoseSample[_correspondingIndex]-p;
	_offsetTcpCalculated = calcOffsetPose(offsetTcpExpected);

}

rw::math::Q ForceStatisticsSingle::calcOffsetPose(const rw::math::Transform3D<> & offset) const
{
	rw::math::RPY<double> r(offset.R());
	rw::math::Vector3D<double> p(offset.P());
	rw::math::Q offsetV(6, p[0], p[2], p[1], r[0], r[1], r[2]);

	std::cout << std::setprecision(10) << std::endl;
	std::cout << dsl::color::GREEN << std::endl;
	DEB(offsetV);
	return offsetV;
}



} /* namespace intelligentmove */
} /* namespace dsl */
