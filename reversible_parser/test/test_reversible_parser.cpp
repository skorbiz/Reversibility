/*
 * test_epuck_driver_extended.cpp
 *
 *  Created on: Nov 20, 2016
 *      Author: josl
 */

// Bring in my package's API, which is what I'm testing
//#include "../include/my_t_framework/my_t_framework.h"
// Bring in gtest

#include <gtest/gtest.h>

double add(int i, int j)
{
	return i + j;
}


// Declare another test
TEST(MyTestCaseName, MyTestName)
{
  EXPECT_EQ(2, add( 1,1));
  EXPECT_EQ(3, add(-1,4));
  EXPECT_EQ(4, add( 2,2));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




