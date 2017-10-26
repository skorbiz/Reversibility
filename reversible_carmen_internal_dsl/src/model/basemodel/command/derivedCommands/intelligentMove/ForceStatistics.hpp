/*
 * ForceStatistics.hpp
 *
 *  Created on: Mar 3, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_FORCESTATISTICS_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_FORCESTATISTICS_HPP_

#include <rw/math/Q.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>

namespace dsl {
namespace intelligentmove {

class ForceStatistics {
public:
	ForceStatistics( std::vector<Record> datasets );
	virtual ~ForceStatistics();

	void reset();
	void update(rw::math::Q qCurrent);
	void useDefaultVariance(bool value);

	rw::math::Q getMeanForceDiviation(const rw::math::Q & force);
	rw::math::Q getMaxForceDiviation(const rw::math::Q & force);
	rw::math::Q getMinForceDiviation(const rw::math::Q & force);
	std::vector<double> getDependentDiviation(const rw::math::Q & meanDiviations);

	unsigned int sizeDataset();

private:
	void updateForce();
	void updateCurrent();
	void updateAdditionalIndexInfo();

	std::vector<unsigned int> calcCorrespondingIndexes() const;
	unsigned int calcCorrespondingIndex(const rw::math::Q & qCurrent, unsigned int indexStart, const Record & recordedData) const;

private:
	/* const */ std::vector<Record> _datasets;
	/* const */ int _searchWindowSize;

	static const double _defaultStdDiviationForceNorm;
	static const double _defaultStdDiviationCurrentNorm;
	bool _useDefaultVraince;



public:
	std::vector<unsigned int> _correspondingIndexes;
	rw::math::Q _qCurrent;

	std::vector<rw::math::Q> _samplesForce;
	rw::math::Q _meanForce;
	rw::math::Q _medianForce;
	rw::math::Q _maxForce;
	rw::math::Q _minForce;
	rw::math::Q _varianceForce;
	rw::math::Q _stdDiviationForce;

	std::vector<rw::math::Q> _samplesCurrent;
	rw::math::Q _meanCurrent;
	rw::math::Q _medianCurrent;
	rw::math::Q _maxCurrent;
	rw::math::Q _minCurrent;
	rw::math::Q _varianceCurrent;
	rw::math::Q _stdDiviationCurrent;

	std::vector<double> _corrIndexPercantage;
	std::vector<double> _corrIndexQDistance;

};


} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_FORCESTATISTICS_HPP_ */


//public:
//	double calcMean() const;
//	double calcVariance() const;
//	double calcMax() const;
//	double calcMin() const;
