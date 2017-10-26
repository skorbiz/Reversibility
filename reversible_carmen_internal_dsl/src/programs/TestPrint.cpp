/*
 * TestPrint.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: josl
 */

#include "TestPrint.hpp"

#include <iomanip>
#include <iostream>

namespace dsl {

TestPrint::TestPrint()
{
	double numbers[] = {1, 1.1, 10.1, 10.11, 100.11, 100.111};

	for(int i = 0; i < 6; i++)
		std::cout << numbers[i] << std::endl;

	std::cout << "-----" << std::endl;
	std::cout << std::setprecision(5);
	for(int i = 0; i < 6; i++)
		std::cout << numbers[i] << std::endl;

	std::cout << "-----" << std::endl;
	std::cout << std::setprecision(5);
	std::cout << std::fixed;
	for(int i = 0; i < 6; i++)
		std::cout << numbers[i] << std::endl;

	std::cout << "-----" << std::endl;
	std::cout << std::setprecision(5);
	std::cout << std::fixed;
	for(int i = 0; i < 6; i++)
		std::cout << std::setw(10) << numbers[i] << std::endl;
}

TestPrint::~TestPrint()
{
}

} /* namespace dsl */
