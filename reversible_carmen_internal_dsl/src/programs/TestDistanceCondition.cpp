/*
 * TestDistanceCondition.cpp
 *
 *  Created on: Jun 9, 2015
 *      Author: josl
 */

#include "TestDistanceCondition.hpp"

namespace dsl {
namespace common{

TestDistanceCondition::TestDistanceCondition()
{
//	positionTestJoin();
	positionTestCartesian();
}

TestDistanceCondition::~TestDistanceCondition()
{
}

void TestDistanceCondition::positionTestJoin()
{
	std::cerr << "Test needs intercace updata" << std::endl;
	exit(-1);
//	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
//	_nodeHnd = rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle);
//	std::string devname = "ur5";
//
//	std::shared_ptr<RobotGeneralInterfaceProxy> RGIP(new RobotGeneralInterfaceProxy(_nodeHnd, devname));
//
//	rw::math::Q pos_screw_table_common (6, 1.162823200251689, -1.0423880870039357, 1.495871336183126, -0.48769007515439944, 2.7459487962869535, -3.1854630658766134);
//	rw::math::Q pos_screwdriver_take_infront(6, 1.0019941733809739, -0.9257137128081577, 1.4554540934745521, -0.5414931210429506, 2.5518927604385513, -3.1537604886637745 );
//
//	dsl::PositionJointComparison c(pos_screwdriver_take_infront, 0.1, RGIP);
//
//	std::cout << "Distance condition test start" << std::endl;
//	RGIP->moveL(pos_screwdriver_take_infront,0.2,0.2);
//	while(true)
//	{
//		c.evaluate();
//	}
//
//
//	std::cout << "test" << std::endl;

}


void TestDistanceCondition::positionTestCartesian()
{
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
	_nodeHnd = rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle);
	std::string devname = "ur5";
	std::shared_ptr<RobotGeneralInterfaceProxy> RGIP(new RobotGeneralInterfaceProxy(_nodeHnd, devname));

	rw::math::Q pos_screw_table_common (6, 1.162823200251689, -1.0423880870039357, 1.495871336183126, -0.48769007515439944, 2.7459487962869535, -3.1854630658766134);
	rw::math::Q pos_screwdriver_take_infront(6, 1.0019941733809739, -0.9257137128081577, 1.4554540934745521, -0.5414931210429506, 2.5518927604385513, -3.1537604886637745 );

	dsl::PositionCartesianComparison c(pos_screwdriver_take_infront, 0.1, RGIP);

	std::cout << "Distance condition test start" << std::endl;
	RGIP->moveL(pos_screwdriver_take_infront,0.2,0.2);
	while(true)
	{
		c.evaluate();
	}

	std::cout << "test" << std::endl;
}




} /* namespace common */
} /* namespace dsl */
