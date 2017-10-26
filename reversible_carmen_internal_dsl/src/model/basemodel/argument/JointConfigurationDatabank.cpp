/*
 * JointConfigurationDatabank.cpp
 *
 *  Created on: Jun 3, 2016
 *      Author: josl
 */

#include "JointConfigurationDatabank.hpp"

namespace dsl {

const std::string JointConfigurationDatabank::jointConfigurationID("jointconfiguration");
const std::string JointConfigurationDatabank::qConfID("qConf");
const std::string JointConfigurationDatabank::qNameID("qName");


JointConfigurationDatabank::JointConfigurationDatabank()
{
}

JointConfigurationDatabank::JointConfigurationDatabank(std::string aFilepath) :
		filepath(aFilepath)
{
	load();
}

JointConfigurationDatabank::~JointConfigurationDatabank()
{
}

// **********************************
// *** BASICS

void JointConfigurationDatabank::add(std::string name, rw::math::Q q)
{
	if(name.empty())
		return promtEmptyName(q);

	if(q.size() == 0)
		return promtEmptyConfig(name);

	if(exists(name))
		return promtNameAlreadyExist();

	if(exists(q))
		return promtConfigAlreadyExist();

	data[name] = q;
}


void JointConfigurationDatabank::remove(std::string name)
{
	if(exists(name))
		data.erase(name);
	else
		std::cerr << "No configuration with name \"" << name << "\" existed." << std::endl;
}

void JointConfigurationDatabank::update(std::string name, rw::math::Q q)
{
	if(exists(name))
		data[name] = q;
	else
		std::cerr << "No configuration with name \"" << name << "\" existed." << std::endl;
}

void JointConfigurationDatabank::rename(std::string nameOrg, std::string nameNew)
{
	if(exists(nameOrg))
	{
		add(nameNew, getQ(nameOrg));
		remove(nameOrg);
	}
	else
		std::cerr << "No configuration with name \"" << nameOrg << "\" existed." << std::endl;
}

rw::math::Q JointConfigurationDatabank::getQ(std::string name)
{
	if(exists(name))
		return 	data[name];
	std::cerr << "No configuration with name \"" << name << "\" exists." << std::endl;
	exit(-1);
}

std::string JointConfigurationDatabank::getName(rw::math::Q q)
{
	typename std::map<std::string, rw::math::Q>::const_iterator iter;
	for (iter = data.begin(); iter != data.end(); iter++)
		if(iter->second == q)
			return iter->first;
	return "no_match_in_database";
}

bool JointConfigurationDatabank::exists(std::string name)
{
	if(data.count(name))
		return false;
	return true;
}

bool JointConfigurationDatabank::exists(rw::math::Q q)
{
	typename std::map<std::string, rw::math::Q>::const_iterator iter;
	for (iter = data.begin(); iter != data.end(); iter++)
		if(iter->second == q)
			return true;
	return false;
}



// **********************************
// *** PROMTS

void JointConfigurationDatabank::promtEmptyName(rw::math::Q q)
{
	std::cerr << "JointConfigurationDatabank::promtEmptyName" << std::endl;
	add(generateRandomName(), q);
}

void JointConfigurationDatabank::promtEmptyConfig(std::string name)
{
	std::cerr << "JointConfigurationDatabank::promtEmptyName" << std::endl;
}

void JointConfigurationDatabank::promtNameAlreadyExist()
{
	std::cerr << "JointConfigurationDatabank::promtNameAlreadyExist" << std::endl;
}

void JointConfigurationDatabank::promtConfigAlreadyExist()
{
	std::cerr << "JointConfigurationDatabank::promtConfigAlreadyExist" << std::endl;
}

// **********************************
// *** OTHER

std::string JointConfigurationDatabank::generateRandomName()
{
	return "rname" + std::to_string(std::rand() % 100000);
}


std::ostream& operator<<(std::ostream& os, const JointConfigurationDatabank& model)
{
	os << "JointConfigurationDatabank contains number of elements: " << model.data.size() << std::endl;
	typename std::map<std::string, rw::math::Q>::const_iterator iter;
	for (iter = model.data.begin(); iter != model.data.end(); iter++)
	{
		os << "Key: " << iter->first << "\t";
		os << "value: " <<iter->second << std::endl;
	}
	return os;
}


// **********************************
// *** SAVE

void JointConfigurationDatabank::save()
{
    rw::common::DOMParser::Ptr parser = rw::common::DOMParser::make();
	rw::common::DOMElem::Ptr doc = parser->getRootElement();

	typename std::map<std::string, rw::math::Q>::const_iterator iter;
	for (iter = data.begin(); iter != data.end(); iter++)
		createRecordDOMElem(iter->first, iter->second,doc);

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


void JointConfigurationDatabank::createRecordDOMElem(const std::string & name, const rw::math::Q & q, rw::common::DOMElem::Ptr doc)
{
	rw::common::DOMElem::Ptr root = doc->addChild(jointConfigurationID);
	rw::common::DOMElem::Ptr child;

	child = root->addChild(qNameID);
	rw::loaders::DOMBasisTypes::createString(name,child);

	child = root->addChild(qConfID);
	rw::loaders::DOMBasisTypes::createQ(q,child);
}


// **********************************
// *** LOAD

void JointConfigurationDatabank::load()
{
	assert(!filepath.empty());

    rw::common::DOMParser::Ptr parser = rw::common::DOMParser::make();
    parser->load(filepath);
    rw::common::DOMElem::Ptr root = parser->getRootElement();

	BOOST_FOREACH( rw::common::DOMElem::Ptr child, root->getChildren() )
	{
		if (child->isName(jointConfigurationID))
			load(child);
		else
			RW_THROW("Parse Error: data value \"" << child->getName() << "\" not recognized");
	}
}

void JointConfigurationDatabank::load(rw::common::DOMElem::Ptr element)
{
	assert(element->getName() == jointConfigurationID);

	BOOST_FOREACH( rw::common::DOMElem::Ptr child, element->getChildren() )
	{
		std::string name;
		rw::math::Q q;

		if (child->isName(qNameID))
			name = rw::loaders::DOMBasisTypes::readString(child->getChild(rw::loaders::DOMBasisTypes::StringId),true);
		else if (child->isName(qConfID))
			q = rw::loaders::DOMBasisTypes::readQ(child->getChild(rw::loaders::DOMBasisTypes::QId),true);
		else
			RW_THROW("Parse Error: data value \"" << child->getName() << "\" not recognized in a Recording");

		add(name,q);
	}

	//ADD new joint conf
}


} /* namespace dsl */
