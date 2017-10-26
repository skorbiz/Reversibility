/*
 * XMLRecordLoader.hpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDLOADER_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDLOADER_HPP_

#include <iostream>
#include <rw/common/DOMElem.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.hpp>
#include "XMLRecordTypes.hpp"

namespace dsl {
namespace intelligentmove {

class XMLRecordLoader {

public:
	XMLRecordLoader();
	virtual ~XMLRecordLoader();

	static RecordDataset load(const std::string& filename);

private:
	static RecordDataset loadDataset(rw::common::DOMElem::Ptr element);
	static Record loadRecord(rw::common::DOMElem::Ptr element);

	static std::vector<rw::math::Q> loadQList(rw::common::DOMElem::Ptr element);
	static Record::MoveType loadType(rw::common::DOMElem::Ptr element);
};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDLOADER_HPP_ */
