///*
// * MinimumTresholdCalculator.hpp
// *
// *  Created on: Mar 23, 2016
// *      Author: josl
// */
//
//#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_MINIMUMTRESHOLDCALCULATOR_HPP_
//#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_MINIMUMTRESHOLDCALCULATOR_HPP_
//
//#include "ForceStatistics.hpp"
//#include "Record.hpp"
//
//namespace dsl {
//namespace intelligentmove {
//
//class MinimumTresholdCalculator
//{
//
//public:
//	MinimumTresholdCalculator( std::vector<Record> datasets);
//	virtual ~MinimumTresholdCalculator();
//
//	void print();
//
//private:
//	void calc();
//	void toMax(rw::math::Q & max, rw::math::Q & newSample);
//	void printMax();
//	void print(std::string name, rw::math::Q & indv, rw::math::Q & sorted);
//
//	std::vector<Record> _datasets;
//	ForceStatistics _fs;
//
//public:
//	rw::math::Q _maxIndividualDiviation;
//	rw::math::Q _maxSortedDiviation;
//	rw::math::Q _maxIndividualDiviationRA2;
//	rw::math::Q _maxIndividualDiviationRA3;
//	rw::math::Q _maxIndividualDiviationRA4;
//	rw::math::Q _maxIndividualDiviationRA5;
//	rw::math::Q _maxSortedDiviationRA2;
//	rw::math::Q _maxSortedDiviationRA3;
//	rw::math::Q _maxSortedDiviationRA4;
//	rw::math::Q _maxSortedDiviationRA5;
//
//};
//
//} /* namespace intelligentmove */
//} /* namespace dsl */
//
//#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_MINIMUMTRESHOLDCALCULATOR_HPP_ */
