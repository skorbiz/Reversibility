/*
 * JointConfigurationDatabank.hpp
 *
 *  Created on: Jun 3, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_ARGUMENT_JOINTCONFIGURATIONDATABANK_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_ARGUMENT_JOINTCONFIGURATIONDATABANK_HPP_

#include <iostream>
#include <rw/common/DOMElem.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>


namespace dsl
{

class JointConfigurationDatabank
{

public:
	JointConfigurationDatabank();
	JointConfigurationDatabank(std::string filepath);
	virtual ~JointConfigurationDatabank();

	void add(std::string name, rw::math::Q q);
	void remove(std::string name);

	void update(std::string name, rw::math::Q q);
	void rename(std::string nameOrg, std::string nameNew);

	rw::math::Q getQ(std::string name);
	std::string getName(rw::math::Q);
	bool exists(std::string name);
	bool exists(rw::math::Q q);

	void save();

	friend std::ostream& operator<<(std::ostream& os, const JointConfigurationDatabank & model);

private:
	void promtEmptyName(rw::math::Q q);
	void promtEmptyConfig(std::string name);
	void promtNoRecodExists();
	void promtNameAlreadyExist();
	void promtConfigAlreadyExist();

	std::string generateRandomName();


	void createRecordDOMElem(const std::string & name, const rw::math::Q & q, rw::common::DOMElem::Ptr doc);
	void load();
	void load(rw::common::DOMElem::Ptr element);



private:
    static const std::string jointConfigurationID;
    static const std::string qConfID;
    static const std::string qNameID;

	std::map<std::string, rw::math::Q> data;
	std::string filepath;

};

} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_ARGUMENT_JOINTCONFIGURATIONDATABANK_HPP_ */
