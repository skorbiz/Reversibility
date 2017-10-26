/*
 * Loader.cpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#include "XMLRecordLoader.hpp"

#include <exception>
#include <boost/foreach.hpp>

#include <rw/common/DOMParser.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>


namespace dsl {
namespace intelligentmove {

XMLRecordLoader::XMLRecordLoader()
{
}

XMLRecordLoader::~XMLRecordLoader()
{
}

RecordDataset XMLRecordLoader::load(const std::string& filename)
{

    rw::common::DOMParser::Ptr parser = rw::common::DOMParser::make();
    parser->load(filename);
    rw::common::DOMElem::Ptr root = parser->getRootElement();
    return (loadDataset(root));
}


// **********************************
// *** Root

RecordDataset XMLRecordLoader::loadDataset(rw::common::DOMElem::Ptr element)
{
	RecordDataset dataset;
	BOOST_FOREACH( rw::common::DOMElem::Ptr child, element->getChildren() )
	{
		if (child->isName(XMLRecordTypes::recordID))
			dataset.addRecord( loadRecord(child) );
		else
			RW_THROW("Parse Error: data value \"" << child->getName() << "\" not recognized");
	}
	return dataset;
}

// **********************************
// *** Record

Record XMLRecordLoader::loadRecord(rw::common::DOMElem::Ptr element)
{
	assert(element->getName() == XMLRecordTypes::recordID);

	Record recording;

	BOOST_FOREACH( rw::common::DOMElem::Ptr child, element->getChildren() )
	{
		if (child->isName(XMLRecordTypes::qStartID))
			recording.qStart = rw::loaders::DOMBasisTypes::readQ(child->getChild(rw::loaders::DOMBasisTypes::idQ()),true);
		else if (child->isName(XMLRecordTypes::qEndID))
			recording.qEnd = rw::loaders::DOMBasisTypes::readQ(child->getChild(rw::loaders::DOMBasisTypes::idQ()),true);
		else if (child->isName(XMLRecordTypes::speedID))
			recording.speed = rw::loaders::DOMBasisTypes::readDouble(child->getChild(rw::loaders::DOMBasisTypes::idDouble()),true);
		else if (child->isName(XMLRecordTypes::accelerationID))
			recording.acceleration = rw::loaders::DOMBasisTypes::readDouble(child->getChild(rw::loaders::DOMBasisTypes::idDouble()),true);
		else if (child->isName(XMLRecordTypes::isActiveID))
			recording.isActive = rw::loaders::DOMBasisTypes::readBool(child->getChild(rw::loaders::DOMBasisTypes::idBoolean()),true);
		else if (child->isName(XMLRecordTypes::successfullMoveID))
			recording.successfullMove = rw::loaders::DOMBasisTypes::readBool(child->getChild(rw::loaders::DOMBasisTypes::idBoolean()),true);
		else if (child->isName(XMLRecordTypes::noteID))
			recording.note = rw::loaders::DOMBasisTypes::readString(child->getChild(rw::loaders::DOMBasisTypes::idString()),true);
		else if (child->isName(XMLRecordTypes::qSampleID))
			recording.qSample = loadQList(child);
		else if (child->isName(XMLRecordTypes::tcpPoseSampleID))
			recording.tcpPoseSample = loadQList(child);
		else if (child->isName(XMLRecordTypes::forceSampleID))
			recording.forceSample = loadQList(child);
		else if (child->isName(XMLRecordTypes::currentSampleID))
			recording.iActual = loadQList(child);
		else if (child->isName(XMLRecordTypes::typeID))
			recording.type = loadType(child);
		else
			RW_THROW("Parse Error: data value \"" << child->getName() << "\" not recognized in a Recording");
	}

	assert(recording.qSample.size() == recording.forceSample.size());
	assert(recording.qSample.size() == recording.iActual.size());
	return recording;
}

std::vector<rw::math::Q> XMLRecordLoader::loadQList(rw::common::DOMElem::Ptr element)
{
	std::vector<rw::math::Q> qList;
	BOOST_FOREACH( rw::common::DOMElem::Ptr child, element->getChildren() )
	{
		if (child->isName(rw::loaders::DOMBasisTypes::idQ()))
			qList.push_back( rw::loaders::DOMBasisTypes::readQ(child,true) );
		else
			RW_THROW("Parse Error: data value \"" << child->getName() << "\" not recognized");
	}
	return qList;
}

Record::MoveType XMLRecordLoader::loadType(rw::common::DOMElem::Ptr element)
{
	std::string type = element->getChild(rw::loaders::DOMBasisTypes::idString())->getValue();
	if(	type == XMLRecordTypes::typeMoveID )
		return Record::MoveType::move;
	else if (type == XMLRecordTypes::typeServoID)
		return Record::MoveType::servo;
	else
		RW_THROW("Parse Error: data value \"" << type << "\" not recognized");

	return Record::MoveType::move;
}

} /* namespace intelligentmove */
} /* namespace dsl */
