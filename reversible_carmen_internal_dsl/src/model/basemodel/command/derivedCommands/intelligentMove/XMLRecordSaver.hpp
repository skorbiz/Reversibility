/*
 * XMLRecordSaver.hpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDSAVER_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDSAVER_HPP_

#include <iostream>
#include <rw/common/DOMElem.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.hpp>
#include "XMLRecordTypes.hpp"


namespace dsl {
namespace intelligentmove {

class XMLRecordSaver {
public:
	XMLRecordSaver();
	virtual ~XMLRecordSaver();

	void save(RecordDataset rdset, std::string filepath);
	void createRecordDOMElem(Record rec, rw::common::DOMElem::Ptr doc);
};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDSAVER_HPP_ */
