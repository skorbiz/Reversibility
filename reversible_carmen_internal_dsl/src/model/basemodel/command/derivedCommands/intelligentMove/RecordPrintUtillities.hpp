/*
 * RecordPrintUtillities.hpp
 *
 *  Created on: Mar 9, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORDPRINTUTILLITIES_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORDPRINTUTILLITIES_HPP_


#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.hpp>
#include "ForceStatistics.hpp"


namespace dsl {
namespace intelligentmove {

class RecordPrintUtillities {
public:
	RecordPrintUtillities();
	virtual ~RecordPrintUtillities();
	static void printToFile(std::string filepath, const Record & r);
	static void printToFile(std::string filepath, const Record & r, RecordDataset & rd);

private:
	static void printToFile(std::string filepath, const std::vector<std::string> & headings, const std::vector<std::vector<double> > & values);
	static void printAddBasics(const Record & r, std::vector<std::string> & headings, std::vector<std::vector<double> > & values);
	static void printAddAditionalInfo(const Record & r, std::vector<std::string> & headings, std::vector<std::vector<double> > & values);
	static void printAddForceStats(const Record & r, std::vector<std::string> & headings, std::vector<std::vector<double> > & values, ForceStatistics & fs);

	static void printAdd6DVec(std::string name, const std::vector<rw::math::Q> & input, std::vector<std::string> & headings, std::vector<std::vector<double> > & values);


};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_RECORDPRINTUTILLITIES_HPP_ */
