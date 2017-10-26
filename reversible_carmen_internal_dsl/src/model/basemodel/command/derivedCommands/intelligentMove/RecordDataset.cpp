/*
 * RecordDataset.cpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#include "RecordDataset.hpp"
#include <color/colors.hpp>

namespace dsl {
namespace intelligentmove {

RecordDataset::RecordDataset()
{
}

RecordDataset::~RecordDataset()
{
}

void RecordDataset::addRecord(Record rec)
{
	records.push_back(rec);
}

std::vector<Record> RecordDataset::getRecordsWith(rw::math::Q qStart, rw::math::Q qEnd)
{
	std::cout << "Requesting records with: " << std::endl;
	std::cout << "qStart: " << qStart;
	std::cout << "  |  qEnd: " << qEnd << std::endl;

	std::vector<Record> matches;
	for(unsigned int i = 0; i < records.size(); i++)
	{
		double matchQS = (qStart - records[i].qStart).norm2();
		double matchQE = (qEnd - records[i].qEnd).norm2();

		std::cout << "rStart: " << records[i].qStart;
		std::cout << "  |  rEnd: " << records[i].qEnd;
		std::cout << "  |   "<< matchQS << " | " <<  matchQE << std::endl;

		if(matchQS < 0.001)
			if(matchQE < 0.001)
				matches.push_back(records[i]);

	}
	std::cout << "Totoal number of records found: " << records.size() <<std::endl;
	std::cout << "Matching number of records found: " << matches.size() <<std::endl;
	return matches;


}

std::vector<Record> RecordDataset::getAllRecords()
{
	return records;
}

unsigned int RecordDataset::getSize()
{
	return records.size();
}



} /* namespace intelligentmove */
} /* namespace dsl */
