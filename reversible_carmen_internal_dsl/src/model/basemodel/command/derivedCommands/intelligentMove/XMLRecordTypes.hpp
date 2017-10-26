/*
 * XMLRecordTypes.hpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDTYPES_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDTYPES_HPP_

#include <iostream>

namespace dsl {
namespace intelligentmove {

class XMLRecordTypes
{

public:
    static const std::string recordID;

    static const std::string qStartID;
    static const std::string qEndID;
    static const std::string forceSampleID;
    static const std::string qSampleID;
    static const std::string tcpPoseSampleID;
    static const std::string currentSampleID;
    static const std::string speedID;
    static const std::string accelerationID;
    static const std::string typeID;
    static const std::string isActiveID;
    static const std::string successfullMoveID;
    static const std::string noteID;

    static const std::string typeMoveID;
    static const std::string typeServoID;

};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_XMLRECORDTYPES_HPP_ */
