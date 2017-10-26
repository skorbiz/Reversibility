/*
 * DataPrint.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: josl
 */

#include "DataPrint.hpp"

namespace dsl {
namespace common {

DataPrint::DataPrint(std::string filePath, std::string fileName)
{
	this->filePath = filePath;
	this->fileName = fileName;
	_outputFile.open(filePath + fileName);

	if (_outputFile.is_open() == false)
	{
		std::cerr << "Failed to open file in DataPrint" << std::endl;
		std::cerr << "FileName: " << fileName << std::endl;
		std::cerr << "FilePath: " << filePath << std::endl;
		exit(-1);
	}
}

void DataPrint::writeForceHeader()
{
	_outputFile << "force1 force2 force3 torque1 torque2 torque3 "
			"norm normForce maxForce normTorque maxTorque" << std::endl;
}

void DataPrint::writeForce(const rw::math::Q force)
{
	if(force.size() != 6){
		std::cout << "Force vector size was != 6. Measurement not printed to file" << std::endl;
		return;
	}

	double norm = force.norm2();
	double normForce = std::sqrt(force(0)*force(0) + force(1)*force(1) + force(2)*force(2));
	double normTorque = std::sqrt(force(3)*force(3) + force(4)*force(4) + force(5)*force(5));
	double maxForce = std::abs(force(0));
	double maxTorque = std::abs(force(3));
	if(maxForce < std::abs(force(1))) maxForce = std::abs(force(1));
	if(maxForce < std::abs(force(2))) maxForce = std::abs(force(2));
	if(maxTorque < std::abs(force(4))) maxTorque = std::abs(force(4));
	if(maxTorque < std::abs(force(5))) maxTorque = std::abs(force(5));

	_outputFile << force(0) << " ";
	_outputFile << force(1) << " ";
	_outputFile << force(2) << " ";
	_outputFile << force(3) << " ";
	_outputFile << force(4) << " ";
	_outputFile << force(5) << " ";
	_outputFile << norm 		<< " ";
	_outputFile << normForce 	<< " ";
	_outputFile << maxForce 	<< " ";
	_outputFile << normTorque 	<< " ";
	_outputFile << maxTorque 	<< std::endl;
}


void DataPrint::writeForceExtendedHeader()
{
	_outputFile << "force1 force2 force3 torque1 torque2 torque3 "
			"norm normForce maxForce normTorque maxTorque "
			"q1 q2 q3 q4 q5 q6 "
			"dq1 dq2 dq3 dq4 dq5 dq6 "
			"dqNorm time speed" << std::endl;
}

void DataPrint::writeForceExtended(const rw::math::Q force, const rw::math::Q q, const rw::math::Q dq, double time, double speed)
{
	if(force.size() != 6){
		std::cout << "Force vector size was != 6. Measurement not printed to file" << std::endl;
		return;
	}

	if(q.size() != 6){
		std::cout << "Q vector size was != 6. Measurement not printed to file" << std::endl;
		return;
	}

	if(dq.size() != 6){
		std::cout << "dQ vector size was != 6. Measurement not printed to file" << std::endl;
		return;
	}


	double norm = force.norm2();
	double normForce = std::sqrt(force(0)*force(0) + force(1)*force(1) + force(2)*force(2));
	double normTorque = std::sqrt(force(3)*force(3) + force(4)*force(4) + force(5)*force(5));
	double maxForce = std::abs(force(0));
	double maxTorque = std::abs(force(3));
	if(maxForce < std::abs(force(1))) maxForce = std::abs(force(1));
	if(maxForce < std::abs(force(2))) maxForce = std::abs(force(2));
	if(maxTorque < std::abs(force(4))) maxTorque = std::abs(force(4));
	if(maxTorque < std::abs(force(5))) maxTorque = std::abs(force(5));

	_outputFile << force(0) << " ";
	_outputFile << force(1) << " ";
	_outputFile << force(2) << " ";
	_outputFile << force(3) << " ";
	_outputFile << force(4) << " ";
	_outputFile << force(5) << " ";
	_outputFile << norm 		<< " ";
	_outputFile << normForce 	<< " ";
	_outputFile << maxForce 	<< " ";
	_outputFile << normTorque 	<< " ";
	_outputFile << maxTorque 	<< " ";


	_outputFile << q(0) << " ";
	_outputFile << q(1) << " ";
	_outputFile << q(2) << " ";
	_outputFile << q(3) << " ";
	_outputFile << q(4) << " ";
	_outputFile << q(5) << " ";

	_outputFile << dq(0) << " ";
	_outputFile << dq(1) << " ";
	_outputFile << dq(2) << " ";
	_outputFile << dq(3) << " ";
	_outputFile << dq(4) << " ";
	_outputFile << dq(5) << " ";

	double dqNorm = dq.norm2();
	_outputFile << dqNorm << " ";
	_outputFile << time << " ";
	_outputFile << speed << std::endl;
}



DataPrint::~DataPrint()
{
	_outputFile.close();
}

} /* namespace common */
} /* namespace dsl */
