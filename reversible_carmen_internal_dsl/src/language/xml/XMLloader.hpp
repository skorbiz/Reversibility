/*
 * XMLloader.hpp
 *
 *  Created on: Oct 19, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_XML_XMLLOADER_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_XML_XMLLOADER_HPP_

#include <iostream>
#include <rw/common/DOMElem.hpp>
#include <rw/math.hpp>
#include <color/colors.hpp>
#include <debug/Debug.hpp>
#include <../src/model/basemodel/BaseProgram.hpp>
#include <../src/model/basemodel/argument/quantities/IOPorts.hpp>
#include <../src/model/expandedmodel/MemoryModel.hpp>
#include <../src/language/SequenceBuilder.hpp>

namespace dsl
{

class XMLloader
{

public:
	XMLloader();
	virtual ~XMLloader();
	dsl::BaseProgram load();
	dsl::BaseProgram load(std::string filepath);


private:
	void loadFile(const std::string& filename, const std::string& schemaFileName = "");
	void loadProgram(rw::common::DOMElem::Ptr element);
	void loadSequence(rw::common::DOMElem::Ptr element);

	//Flow manipulations
	void loadCall(rw::common::DOMElem::Ptr element);
	void loadUncall(rw::common::DOMElem::Ptr element);
	void loadFlowOverwrites(rw::common::DOMElem::Ptr element);
	void loadAttributeReversible(rw::common::DOMElem::Ptr attribute);
	void loadAttributeReverewWith(rw::common::DOMElem::Ptr attribute);

	//Sequence commands
	void loadGrasp(rw::common::DOMElem::Ptr element);
	void loadMove(rw::common::DOMElem::Ptr element);
	void loadWait(rw::common::DOMElem::Ptr element);
	void loadIO(rw::common::DOMElem::Ptr element);
	void loadPrint(rw::common::DOMElem::Ptr element);
	void loadForceMode(rw::common::DOMElem::Ptr element);
	void loadAction(rw::common::DOMElem::Ptr element);

	//Save to memory
	void loadQ(rw::common::DOMElem::Ptr element);
	void loadIOPorts(rw::common::DOMElem::Ptr element);

	//Load from memory or simple datatypes
	dsl::IOPorts getPort(rw::common::DOMElem::Ptr element);
	dsl::Switch  getSwitch(rw::common::DOMElem::Ptr element);
	dsl::ForceModeArgument getFMA(rw::common::DOMElem::Ptr element);


private:
    static const std::string sequenceID;

    static const std::string callID;
    static const std::string uncallID;
    static const std::string reverseWithID;
    static const std::string neverReversibleID;

    static const std::string moveID;
    static const std::string waitID;
    static const std::string ioID;
    static const std::string printID;
    static const std::string grapsID;
    static const std::string forceModeID;
    static const std::string actionID;

    static const std::string graspOpenID;
    static const std::string graspCloseID;
    static const std::string fmaForwardID;
    static const std::string fmaBackwardID;
    static const std::string switchOnID;
    static const std::string switchOffID;

    static const std::string nameID;
    static const std::string ioPortID;
    static const std::string portID;
    static const std::string switchID;

private:
    SequenceBuilder _builder;
    MemoryModel<rw::math::Q> _qMem;
    MemoryModel<dsl::IOPorts> _iopMem;
    dsl::debug::Debug deb;

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_XML_XMLLOADER_HPP_ */
