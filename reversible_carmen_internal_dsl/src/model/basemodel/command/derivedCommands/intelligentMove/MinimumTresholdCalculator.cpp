///*
// * MinimumTresholdCalculator.cpp
// *
// *  Created on: Mar 23, 2016
// *      Author: josl
// */
//
//#include "MinimumTresholdCalculator.hpp"
//
//#include <iomanip>
//#include "Statistics.hpp"
//#include <color/colors.hpp>
//
//namespace dsl {
//namespace intelligentmove {
//
//MinimumTresholdCalculator::MinimumTresholdCalculator( std::vector<Record> datasets) :
//		_datasets(datasets),
//		_fs(datasets),
//		_maxIndividualDiviation(6, 0.0),
//		_maxSortedDiviation(6, 0.0),
//		_maxIndividualDiviationRA2(6, 0.0),
//		_maxIndividualDiviationRA3(6, 0.0),
//		_maxIndividualDiviationRA4(6, 0.0),
//		_maxIndividualDiviationRA5(6, 0.0),
//		_maxSortedDiviationRA2(6, 0.0),
//		_maxSortedDiviationRA3(6, 0.0),
//		_maxSortedDiviationRA4(6, 0.0),
//		_maxSortedDiviationRA5(6, 0.0)
//{
//	calc();
//}
//
//MinimumTresholdCalculator::~MinimumTresholdCalculator()
//{
//}
//
//void MinimumTresholdCalculator::calc()
//{
//
//	std::vector<rw::math::Q> diviation;
//
//	std::cout << "min treshold calc" << std::endl;
//	std::cout << "n datasets: " << _datasets.size() << std::endl;
//	for(unsigned int d = 0; d < _datasets.size(); d++)
//	{
//		std::cout << "n sample size: " << _datasets[d].qSample.size() << std::endl;
//
//		_fs.reset();
//		for(unsigned int i = 0; i < _datasets[d].qSample.size(); i++)
//		{
//
//			_fs.update(_datasets[d].qSample[i]);
//
//			diviation.push_back(_fs.getMeanForceDiviation(_datasets[d].forceSample[i]));
//			rw::math::Q dependentDiviations(_fs.getDependentDiviation(diviation.back()));
//
//			toMax(_maxIndividualDiviation, diviation.back());
//			toMax(_maxSortedDiviation, dependentDiviations);
//
//			rw::math::Q ra2Div = Statistics::calcBackwardsAccumulation(diviation,2)/2;
//			rw::math::Q ra3Div = Statistics::calcBackwardsAccumulation(diviation,3)/3;
//			rw::math::Q ra4Div = Statistics::calcBackwardsAccumulation(diviation,4)/4;
//			rw::math::Q ra5Div = Statistics::calcBackwardsAccumulation(diviation,5)/5;
//			rw::math::Q ra2DepDiv(_fs.getDependentDiviation(ra2Div));
//			rw::math::Q ra3DepDiv(_fs.getDependentDiviation(ra3Div));
//			rw::math::Q ra4DepDiv(_fs.getDependentDiviation(ra4Div));
//			rw::math::Q ra5DepDiv(_fs.getDependentDiviation(ra5Div));
//
//			toMax(_maxIndividualDiviationRA2, ra2Div);
//			toMax(_maxIndividualDiviationRA3, ra3Div);
//			toMax(_maxIndividualDiviationRA4, ra4Div);
//			toMax(_maxIndividualDiviationRA5, ra5Div);
//			toMax(_maxSortedDiviationRA2, ra2DepDiv);
//			toMax(_maxSortedDiviationRA3, ra3DepDiv);
//			toMax(_maxSortedDiviationRA4, ra4DepDiv);
//			toMax(_maxSortedDiviationRA5, ra5DepDiv);
//
//			printMax();
//		}
//	}
//}
//
//void MinimumTresholdCalculator::toMax(rw::math::Q & max, rw::math::Q & newSample)
//{
//	for(unsigned int i = 0; i < 6; i++)
//		if(std::abs(max[i]) < std::abs(newSample[i]) )
//			max[i] = newSample[i];
//}
//
//
//void colorfunc(double v, double th1, double th2, double th3);
//void out(double v);
//void out1(double v);
//
//void MinimumTresholdCalculator::printMax()
//{
//	for(unsigned int i = 0; i < 6; i++) out1(_maxIndividualDiviation[i]); std::cout << " | ";
//	for(unsigned int i = 0; i < 6; i++) out1(_maxSortedDiviation[i]); std::cout << " | ";
////	for(unsigned int i = 0; i < 6; i++) out1(_maxIndividualDiviationRA3[i]); std::cout << " | ";
////	for(unsigned int i = 0; i < 6; i++) out1(_maxSortedDiviationRA3[i]); std::cout << " | ";
////	for(unsigned int i = 0; i < _fs._corrIndexPercantage.size(); i++) out1(_fs._corrIndexPercantage[i]); std::cout << " | ";
//	std::cout << std::endl;
//}
//
//
////void MinimumTresholdCalculator::calcUpdated()
////{
////
////
////	for(unsigned int d = 0; d < _datasets.size(); d++)
////	{
////		std::vector<rw::math::Q> diviation;
////		std::vector<std::vector<double> > diviationSorted;
////
////		_fs.reset();
////		Record r = _datasets[d];
////
////		for(unsigned int d = 0; d < r.qSample.size(); d++)
////		{
////			_fs.update(r.qSample[d]);
////			diviation.push_back(_fs.getMeanForceDiviation(r.forceSample[d]));
////			diviation.push_back(_fs.getDependentDiviation(diviation.back()));
////		}
////
//////		std::vector<std::vector<rw::math::Q> > raN;
//////		for(int i = 1; i < 7; i++)
//////			raN.push_back(Statistics::calcRunningAvargeSimple(diviation,1));
////
////		for(unsigned int d = 0; d < r.qSample.size(); d++)
////		{
////			for(unsigned int i = 0; i < 6; i++)
////				if(std::abs(_maxIndividualDiviation[d]) < std::abs(diviation[d][i]) )
////					_maxIndividualDiviation[i] = diviation[i];
////
////			for(unsigned int i = 0; i < 6; i++)
////				if(_maxSortedDiviation[i] < diviationSorted[d][i] )
////					_maxSortedDiviation[i] = diviationSorted[d][i];
////		}
////	}
////
////
////}
//
//
//
//
//void MinimumTresholdCalculator::print()
//{
//	std::cout << "Minimum Treshold calculations: " << std::endl;
//	print("RA1", _maxIndividualDiviation, _maxSortedDiviation);
//	print("RA2", _maxIndividualDiviationRA2, _maxSortedDiviationRA2);
//	print("RA3", _maxIndividualDiviationRA3, _maxSortedDiviationRA3);
//	print("RA4", _maxIndividualDiviationRA4, _maxSortedDiviationRA4);
//	print("RA5", _maxIndividualDiviationRA5, _maxSortedDiviationRA5);
//}
//
//void MinimumTresholdCalculator::print(std::string name, rw::math::Q & indv, rw::math::Q & sorted)
//{
//	std::cout << name  << " : ";
//	std::cout << " individual: ";
//	for(unsigned int i = 0; i < indv.size(); i++)
//		std::cout << std::setw(9) << indv[i] << ", ";
//	std::cout << "   dependent: ";
//	for(unsigned int i = 0; i < sorted.size(); i++)
//		std::cout << std::setw(9) << sorted[i] << ", ";
//	std::cout << std::endl;
//}
//
//
//} /* namespace intelligentmove */
//} /* namespace dsl */
