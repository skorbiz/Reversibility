/*
 * XMLSaver.cpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#include "XMLRecordSaver.hpp"

#include <fstream>
#include <assert.h>
#include <rw/common/DOMParser.hpp>

namespace dsl {
namespace intelligentmove {

XMLRecordSaver::XMLRecordSaver()
{
}

XMLRecordSaver::~XMLRecordSaver()
{
}

void XMLRecordSaver::save(RecordDataset rdset, std::string filepath)
{
    rw::common::DOMParser::Ptr parser = rw::common::DOMParser::make();
	rw::common::DOMElem::Ptr doc = parser->getRootElement();

	std::vector<Record> rec = rdset.getAllRecords();
	for(unsigned int i = 0; i < rec.size(); i++)
		createRecordDOMElem(rec[i],doc);


	std::ofstream myfile;
	myfile.open (filepath);

	if (myfile.is_open())
	{
		parser->save(myfile);
		myfile.close();
	}
	else
	{
		std::cerr << "Unable to open file to save imove" << std::endl;
		exit(-1);
	}

}


void XMLRecordSaver::createRecordDOMElem(Record rec, rw::common::DOMElem::Ptr doc)
{
	rw::common::DOMElem::Ptr root = doc->addChild(XMLRecordTypes::recordID);
	rw::common::DOMElem::Ptr child;

	child = root->addChild(XMLRecordTypes::qStartID);
	rw::loaders::DOMBasisTypes::createQ(rec.qStart,child);

	child = root->addChild(XMLRecordTypes::qEndID);
	rw::loaders::DOMBasisTypes::createQ(rec.qEnd,child);

	child = root->addChild(XMLRecordTypes::speedID);
	rw::loaders::DOMBasisTypes::createDouble(rec.speed,child);

	child = root->addChild(XMLRecordTypes::accelerationID);
	rw::loaders::DOMBasisTypes::createDouble(rec.acceleration,child);

	child = root->addChild(XMLRecordTypes::isActiveID);
	rw::loaders::DOMBasisTypes::createBoolean(rec.isActive,child);

	child = root->addChild(XMLRecordTypes::successfullMoveID);
	rw::loaders::DOMBasisTypes::createBoolean(rec.successfullMove,child);

	child = root->addChild(XMLRecordTypes::noteID);
	rw::loaders::DOMBasisTypes::createString(rec.note,child);

	std::string typeMove;
	if(rec.type == Record::MoveType::move)
		typeMove = XMLRecordTypes::typeMoveID;
	else if(rec.type == Record::MoveType::servo)
		typeMove = XMLRecordTypes::typeServoID;

	child = root->addChild(XMLRecordTypes::typeID);
	rw::loaders::DOMBasisTypes::createString(typeMove,child);

	assert(rec.qSample.size() == rec.forceSample.size());
	assert(rec.qSample.size() == rec.iActual.size());

	child = root->addChild(XMLRecordTypes::qSampleID);
	for(unsigned int i = 0; i < rec.qSample.size(); i++)
		rw::loaders::DOMBasisTypes::createQ(rec.qSample[i],child);

	child = root->addChild(XMLRecordTypes::tcpPoseSampleID);
	for(unsigned int i = 0; i < rec.tcpPoseSample.size(); i++)
		rw::loaders::DOMBasisTypes::createQ(rec.tcpPoseSample[i],child);

	child = root->addChild(XMLRecordTypes::forceSampleID);
	for(unsigned int i = 0; i < rec.forceSample.size(); i++)
		rw::loaders::DOMBasisTypes::createQ(rec.forceSample[i],child);

	child = root->addChild(XMLRecordTypes::currentSampleID);
	for(unsigned int i = 0; i < rec.iActual.size(); i++)
		rw::loaders::DOMBasisTypes::createQ(rec.iActual[i],child);

}


} /* namespace intelligentmove */
} /* namespace dsl */
