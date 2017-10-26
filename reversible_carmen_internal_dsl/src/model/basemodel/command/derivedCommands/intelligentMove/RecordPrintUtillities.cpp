/*
 * RecordPrint.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: josl
 */

#include "RecordPrintUtillities.hpp"
#include <iomanip>
#include <iostream>

namespace dsl {
namespace intelligentmove {

RecordPrintUtillities::RecordPrintUtillities()
{
}

RecordPrintUtillities::~RecordPrintUtillities()
{
}

void RecordPrintUtillities::printToFile(std::string filepath, const Record & r)
{
	std::vector<std::string> headings;
	std::vector<std::vector<double> > values;
	printAddBasics(r, headings,values);
	printToFile(filepath,headings,values);
}

void RecordPrintUtillities::printToFile(std::string filepath, const Record & r, RecordDataset & rd)
{
	std::vector<std::string> headings;
	std::vector<std::vector<double> > values;
	ForceStatistics fs = rd.getRecordsWith(r.qStart, r.qEnd);
	printAddBasics(r, headings,values);
	printAddForceStats(r, headings,values,fs);
	printToFile(filepath,headings,values);
}

void RecordPrintUtillities::printToFile(std::string filepath, const std::vector<std::string> & headings, const std::vector<std::vector<double> > & values)
{
	assert(false && "fix");
/*	std::ofstream outputFile;
	outputFile.open(filepath);

	if (outputFile.is_open() == false)
	{
		std::cerr << "Failed to open file " << filepath << "in Record print" << std::endl;
		exit(-1);
	}

	for(unsigned int i = 0; i < headings.size(); i++)
		outputFile << headings[i] << " ";
	outputFile << "\n";

	for(unsigned int i = 0; i < values[0].size(); i++)
	{
		for(unsigned int j = 0; j < values.size(); j++)
			outputFile << values[j][i] << " ";
		outputFile << "\n";
	}

	outputFile.close();
	*/
}

void RecordPrintUtillities::printAddBasics(const Record & r, std::vector<std::string> & headings, std::vector<std::vector<double> > & values)
{
	std::cout << "Add Basics" << std::endl;
	assert(r.forceSample.size() == r.tcpPoseSample.size());
	assert(r.forceSample.size() == r.timestamp.size());
	assert(r.forceSample.size() == r.timestampPC.size());
	assert(r.forceSample.size() == r.qSample.size());
	assert(r.forceSample.size() == r.iActual.size());
	assert(r.forceSample.size() == r.iControl.size());

	printAdd6DVec("force",r.forceSample, headings, values);
	printAdd6DVec("dforce",Statistics::calcDQ(r.forceSample), headings, values);

	printAdd6DVec("TcpPose", r.tcpPoseSample, headings, values);
	printAdd6DVec("dTcpPose", Statistics::calcDQ(r.tcpPoseSample), headings, values);

	printAdd6DVec("q",r.qSample, headings, values);
	printAdd6DVec("dq",Statistics::calcDQ(r.qSample), headings, values);

	printAdd6DVec("iActual",r.iActual, headings, values);
	printAdd6DVec("iControl",r.iControl, headings, values);

	headings.push_back("sampleN");
	values.push_back(Statistics::getSampleNumbers(r.forceSample.size()));

	headings.push_back("time");
	values.push_back(r.timestamp);

	headings.push_back("timeRelative");
	values.push_back(Statistics::calcRelative(r.timestamp));

	headings.push_back("timePc");
	values.push_back(r.timestampPC);

	headings.push_back("timePcRelative");
	values.push_back(Statistics::calcRelative(r.timestampPC));

	headings.push_back("safetyMode");
	values.push_back(r.safetyMode);

	headings.push_back("errorState");
	values.push_back(r.errorState);
}

void RecordPrintUtillities::printAddAditionalInfo(const Record & r, std::vector<std::string> & headings, std::vector<std::vector<double> > & values)
{
	std::cout << "Additional info" << std::endl;

	printAdd6DVec("forceRA1W", Statistics::calcRunningAvarge(r.forceSample,1), headings, values);
	printAdd6DVec("forceRA2W", Statistics::calcRunningAvarge(r.forceSample,2), headings, values);
	printAdd6DVec("forceRA9W", Statistics::calcRunningAvarge(r.forceSample,9), headings, values);
	printAdd6DVec("forceRA1ScaledW", Statistics::calcRunningAvarge(r.forceSample,1,true), headings, values);
	printAdd6DVec("forceRA2ScaledW", Statistics::calcRunningAvarge(r.forceSample,2,true), headings, values);
	printAdd6DVec("forceRA9ScaledW", Statistics::calcRunningAvarge(r.forceSample,9,true), headings, values);
}


void RecordPrintUtillities::printAddForceStats(const Record & r, std::vector<std::string> & headings, std::vector<std::vector<double> > & values, ForceStatistics & fs)
{
	std::cout << "printAddForceStats" << std::endl;

	fs.reset();

	std::vector<std::vector<rw::math::Q> > valForce(6);
	std::vector<std::vector<rw::math::Q> > valCurrent(6);

	std::vector<std::vector<double> > valIndex;
	std::vector<std::vector<double> > valIndexPercentage;
	std::vector<std::vector<double> > valIndexQDistance;

	std::vector<std::vector<rw::math::Q> > valSamplesForce;
	std::vector<std::vector<rw::math::Q> > valSamplesCurrent;

	for(unsigned int i = 0; i < r.qSample.size(); i++)
	{
		fs.update(r.qSample[i]);

		valForce[0].push_back(fs._meanForce);
		valCurrent[0].push_back(fs._meanCurrent);

		valForce[1].push_back(fs._medianForce);
		valCurrent[1].push_back(fs._medianCurrent);

		valForce[2].push_back(fs._maxForce);
		valCurrent[2].push_back(fs._maxCurrent);

		valForce[3].push_back(fs._minForce);
		valCurrent[3].push_back(fs._minCurrent);

		valForce[4].push_back(fs._stdDiviationForce);
		valCurrent[4].push_back(fs._stdDiviationCurrent);

		valForce[5].push_back(fs._varianceForce);
		valCurrent[5].push_back(fs._varianceCurrent);

		valIndex.push_back(std::vector<double>(fs._correspondingIndexes.begin(), fs._correspondingIndexes.end()));
		valIndexPercentage.push_back(fs._corrIndexPercantage);
		valIndexQDistance.push_back(fs._corrIndexQDistance);

		valSamplesForce.push_back(fs._samplesForce);
		valSamplesCurrent.push_back(fs._samplesCurrent);

	}

	printAdd6DVec("meanForce", 	valForce[0], headings, values);
	printAdd6DVec("medianForce",valForce[1], headings, values);
	printAdd6DVec("maxForce", 	valForce[2], headings, values);
	printAdd6DVec("minForce", 	valForce[3], headings, values);
	printAdd6DVec("stdDForce", 	valForce[4], headings, values);
	printAdd6DVec("varForce", 	valForce[5], headings, values);

	printAdd6DVec("meanCurrent", 	valCurrent[0], headings, values);
	printAdd6DVec("medianCurrent",  valCurrent[1], headings, values);
	printAdd6DVec("maxCurrent", 	valCurrent[2], headings, values);
	printAdd6DVec("minCurrent", 	valCurrent[3], headings, values);
	printAdd6DVec("stdDCurrent", 	valCurrent[4], headings, values);
	printAdd6DVec("varCurrent", 	valCurrent[5], headings, values);


	for(unsigned int i = 0; i < fs.sizeDataset(); i++)
	{
		headings.push_back("corrIndexSet" + std::to_string(i));
		headings.push_back("corrIndexPercentageSet" + std::to_string(i));
		headings.push_back("corrIndexQEndDistSet" + std::to_string(i));

		values.push_back(Statistics::getColumn(valIndex,i));
		values.push_back(Statistics::getColumn(valIndexPercentage,i));
		values.push_back(Statistics::getColumn(valIndexQDistance,i));
	}

	for(unsigned int i = 0; i < fs.sizeDataset(); i++)
		printAdd6DVec("datasetForce" + std::to_string(i) + "Sample",
				Statistics::getColumn(valSamplesForce,i), headings, values);

	for(unsigned int i = 0; i < fs.sizeDataset(); i++)
		printAdd6DVec("datasetCurrent" + std::to_string(i) + "Sample",
				Statistics::getColumn(valSamplesCurrent,i), headings, values);

}

void RecordPrintUtillities::printAdd6DVec(std::string name, const std::vector<rw::math::Q> & input, std::vector<std::string> & headings, std::vector<std::vector<double> > & values)
{
	headings.push_back(name);
	values.push_back(Statistics::calcNorms(input));

	for(unsigned int i = 0; i < 6; i++)
	{
		headings.push_back(name + std::to_string(i));
		values.push_back(Statistics::getColumn(input, i));
	}
}




} /* namespace intelligentmove */
} /* namespace dsl */
