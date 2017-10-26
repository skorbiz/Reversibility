/*
 * D7pcCodeGenneration.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: josl
 */

#include <reversible_parser/code_genneration/CodeGen.h>
#include <reversible_parser/D7pcAstType.h>
#include <D7pcCodeGenneration.h>
#include <reversible_parser/D7pcTokenType.h>
#include <reversible_parser/support/File23.h>
#include <reversible_parser/lexer/Token.h>
#include <reversible_parser/lexer/TokenType.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <vector>

namespace edsl {

D7pcCodeGenneration::D7pcCodeGenneration(std::shared_ptr<abstract_syntax_tree::ASTNode> root)
{
	std::string filepathTemplate = ros::package::getPath("reversible_d7pc_interpreter") + "/src/gennerated_code/";
	std::string filepathOutput = ros::package::getPath("reversible_d7pc_interpreter") + "/src/gennerated_code/";

	{
		std::cout << "Code genneration of hpp file" << std::endl;
		std::string filenameTemplate = "D7pcString2VariableTemplate.h";
		std::string filenameOutput = "D7pcString2Variable.h";
		File23 fileTemplate(filepathTemplate, filenameTemplate);
		std::string stringTemplate = fileTemplate.readFile();
		CodeGen output(stringTemplate, filepathOutput + filenameOutput);
		output.gennerate();
	}

	{
		std::cout << "Code genneration of cpp file" << std::endl;
		std::string filenameTemplate = "D7pcString2VariableTemplate.cpp";
		std::string filenameOutput = "D7pcString2Variable.cpp";
		File23 fileTemplate(filepathTemplate, filenameTemplate);
		std::string stringTemplate = fileTemplate.readFile();
		CodeGen gennerator(stringTemplate, filepathOutput + filenameOutput);


		using namespace edsl::lexer;
		//using namespace edsl::parser;
		using namespace edsl::abstract_syntax_tree;



		//Frames
		if(root->getDecendents(D7pcAstType::AT_FRAMES).size() == 0)
			return;

		std::string frames_replacement_text;
		auto frames_list = root->getSoleDecendent(D7pcAstType::AT_FRAMES);
		auto frames = frames_list->abstractSyntaxTreeChilds;
		for(auto f : frames)
		{
			std::string identifier = toString(f);
			frames_replacement_text += "// if(name == \""+identifier+"\") return " + identifier + ";\n";
		}

		gennerator.addReplacementRule("var1", frames_replacement_text);
		gennerator.addReplacementRule(".", "abc2");

		gennerator.gennerate();
	}


}

D7pcCodeGenneration::~D7pcCodeGenneration()
{
}

std::string D7pcCodeGenneration::toString(std::shared_ptr<abstract_syntax_tree::ASTNode> identifier)
{
	if(identifier->content->isOfType(D7pcTokenType::TT_DOT))
		return "->";

	if(identifier->abstractSyntaxTreeChilds.size() == 0)
		return identifier->getContent();

	std::string output;
	for(auto child : identifier->abstractSyntaxTreeChilds)
		output = output + toString(child);
	return output;

}

} /* namespace edsl */
