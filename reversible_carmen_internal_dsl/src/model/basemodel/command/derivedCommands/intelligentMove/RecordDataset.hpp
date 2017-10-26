/*
 * RecordDataset.hpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORDDATASET_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORDDATASET_HPP_

#include <vector>
#include <rw/math/Q.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>

namespace dsl {
namespace intelligentmove {

class RecordDataset {
public:
	RecordDataset();
	virtual ~RecordDataset();

	void addRecord(Record rec);
	std::vector<Record> getRecordsWith(rw::math::Q qStart, rw::math::Q qEnd);
	std::vector<Record> getAllRecords();

	unsigned int getSize();

//	void save();
//	void saveNew(std::string filepath);
//	void load(std::string filepath);
//	void loadDefault();

private:
	std::vector<Record> records;
};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORDDATASET_HPP_ */
