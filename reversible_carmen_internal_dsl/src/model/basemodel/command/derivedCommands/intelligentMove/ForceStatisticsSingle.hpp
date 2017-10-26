/*
 * ForceStatisticsSingle.hpp
 *
 *  Created on: Jul 6, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_FORCESTATISTICSSINGLE_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_FORCESTATISTICSSINGLE_HPP_

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>
#include <debug/Debug.hpp>

namespace dsl {
namespace intelligentmove {

class ForceStatisticsSingle
{

public:
	ForceStatisticsSingle( std::vector<Record> datasets );
	virtual ~ForceStatisticsSingle();

	void reset();
//	void update(rw::math::Q qCurrent);
	void update(int stategy, rw::math::Q q, rw::math::Q p, rw::math::Q i);
	void calcOffsets(const rw::math::Q & q, const rw::math::Q & p, const rw::math::Transform3D<> & offseTcpExpected);


private:
	void updateAdditionalIndexInfo();
	void updateCurrent();
	std::vector<rw::math::Q> calcSampleCurrents(unsigned int index, const Record & recordedData) const;
	unsigned int calcCorrespondingIndex(const rw::math::Q & currentInput, const std::vector<rw::math::Q>& recordedSamples, unsigned int indexStart, unsigned int windowSize);

	rw::math::Q calcOffsetPose(const rw::math::Transform3D<> & offset) const;

public:
	unsigned int _correspondingIndex;

	std::vector<rw::math::Q> _samplesCurrent;
	rw::math::Q _meanCurrent;
	rw::math::Q _medianCurrent;
	rw::math::Q _maxCurrent;
	rw::math::Q _minCurrent;
	rw::math::Q _varianceCurrent;
	rw::math::Q _stdDiviationCurrent;

	double _corrIndexPercantage;
	double _corrIndexQDistance;
	double _corrIndexBestMatch;
	unsigned int _corrIndexLast;
	unsigned int _corrIndexDirect;


private:
	Record _dataset;
	int _windowSizeSearch;
	int _windowSizeKernal;

	rw::math::Q _offsetQ;
	rw::math::Q _offsetTcpMeasured;
	rw::math::Q _offsetTcpCalculated;



	mutable dsl::debug::Debug deb;


};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_FORCESTATISTICSSINGLE_HPP_ */
