/*
 * TestServo.cpp
 *
 *  Created on: Dec 23, 2015
 *      Author: josl
 */

#include "TestServo.hpp"

//#include <caros/serial_device_si_proxy.h>
//#include <caros/proxy/ur_proxy.h>
#include <rw/math/Q.hpp>

#include <../src/common/DataPrint.hpp>
#include <color/colors.hpp>

namespace dsl
{

//void dslMove(rw::math::Q q, caros::SerialDeviceSIProxy & sip, caros::UrProxy & urp, dsl::common::DataPrint & dp )
//{
////	sip.moveServoQ(q,0.1);
//	sip.movePtp(q, 0.2, 0);
//	double diff=1.0;
//	bool stop = false;
//	while(diff > 0.005)
//	{
//		rw::math::Q qc;
//		qc = sip.getQ();
//		if(qc.size() == 6 )
//			diff = (q-qc).norm2();
//
////		std::cout << diff << std::endl;
//
//		dp.writeForce(urp.getTcpForce());
//
//		rw::math::Q force = urp.getTcpForce();
//		if(force.size() == 6)
//		{
//			double normForce = std::sqrt(force(0)*force(0) + force(1)*force(1) + force(2)*force(2));
//			if(normForce > 40)
//			{
//				urp.stopj(250);
//				stop = true;
//				std::cout << dsl::color::BOLDRED << normForce << dsl::color::DEFAULT << std::endl;
////				std::cout << dsl::color::BOLDRED << "!!! STOPED !!!" << dsl::color::DEFAULT << std::endl;
////				ros::Duration(5).sleep();
//			}
//			else
//			{
//				if(stop == true)
//				{
//					sip.movePtp(q, 0.2, 0);
//					stop = false;
//				}
//				std::cout << normForce << std::endl;
//			}
//		}
//	}
//}
//
//
//void dslMoveDiff(rw::math::Q q, caros::SerialDeviceSIProxy & sip, caros::UrProxy & urp, dsl::common::DataPrint & dp )
//{
//	sip.movePtp(q, 0.2, 0);
//	double diffMin = 999;
//
//	while(true)
//	{
//		double diff = 999;
//		rw::math::Q qc;
//		qc = sip.getQ();
//		if(qc.size() == 6 )
//			diff = (q-qc).norm2();
//
//		if(diffMin > diff)
//			diffMin = diff;
//
//		double normForce = 0;
//		rw::math::Q force = urp.getTcpForce();
//		if(force.size() == 6)
//			normForce = std::sqrt(force(0)*force(0) + force(1)*force(1) + force(2)*force(2));
//
//		std::cout << " Diff: " << diff;
//		std::cout << " diffMin: " << diffMin;
//		std::cout << " force: " << normForce;
//		std::cout << std::endl;
//
//		if(normForce > 60)
//		{
//			std::cout << dsl::color::BOLDRED << "!!! RETURNING !!!" << dsl::color::DEFAULT << std::endl;
//			ros::Duration(5).sleep();
//			break;
//		}
//	}
//}
//
//double timelast = 0;
//
//void dslServo(rw::math::Q qEnd, caros::SerialDeviceSIProxy & sip, caros::UrProxy & urp, dsl::common::DataPrint & dp )
//{
//	rw::math::Q qCLast(6);
//	rw::math::Q qTarget(6);
//	double epsilonQTarget = 0.001;
//	double epsilonQTargetStep = 0.001;
//	double epsilonQTargetMax = 0.01;
//
//	int i = 0;
//	bool doCheck = true;
//	double time = 0;
//	while(true)
//	{
//		rw::math::Q qC(7);
//		while(qC.size() != 6)
//		 qC = sip.getQ();
//
//		double diffToEnd =  (qEnd-qC).norm2();
//		rw::math::Q deltaQ = (qEnd-qC)/diffToEnd*epsilonQTarget;
//
//		double diffToOldTarget =  (qTarget-qC).norm2();
//
//		std::cout << " N" << i++ << " ";
////		std::cout << " diff: " << diffToOldTarget;
////		std::cout << std::endl;
//
////		dp.writeForce(urp.getForce());
//		dp.writeForceExtended(urp.getTcpForce(),qC,qC-qCLast,urp.getControllerTimeStamp(),epsilonQTarget);
//		qCLast = qC;
//
//
////		std::cout << "target pc: " << qTarget;
////		std::cout << " target robot: " << urp.getTargetJointPosition();
////		std::cout << std::endl;
//
////		double time = urp.getControllerTimeStamp();
////		if(time-timelast != 0)
////		{
////		std::cout << " N" << i;
////		std::cout << " time 0: " << time-timelast;
////		std::cout << " time 1 : " << time;
////		std::cout << " time 2: " << urp.getDriverTimeStamp();
////		std::cout << " time 3: " << sip.getTimeStamp();
////		std::cout << std::endl;
////		}
////		i++;
////		timelast = time;
//
//		rw::math::Q force = urp.getTcpForce();
//		if(force.size() == 6)
//		{
//			double normForce = std::sqrt(force(0)*force(0) + force(1)*force(1) + force(2)*force(2));
//			if(normForce > 60 && i > 50 && doCheck)
//			{
//				urp.stopj(50);
//				std::cout << dsl::color::BOLDRED << normForce << dsl::color::DEFAULT << std::endl;
//				doCheck = false;
//				time = urp.getControllerTimeStamp()+3;
//			}
//			else if(urp.getControllerTimeStamp() > time)
//			{
//				qTarget = qC + deltaQ;
//				urp.servoQ(qTarget);
//				std::cout << normForce << std::endl;
//			}
//		}
//
//		if(epsilonQTarget <= epsilonQTargetMax)
//			epsilonQTarget += epsilonQTargetStep;
//
//
//
//		if(epsilonQTarget > diffToEnd)
//		{
//			urp.servoQ(qEnd);
//			std::cout << dsl::color::BOLDRED << "!!! SERVO RETURNING !!!" << dsl::color::DEFAULT << std::endl;
//			ros::Duration(1).sleep();
//			return;
//		}
//	}
//}


TestServo::TestServo()
{
//	std::cout << dsl::color::BOLDMAGENTA << "Program start";
//	std::cout << dsl::color::DEFAULT << std::endl;
//
//	ros::NodeHandle nh;
//	caros::SerialDeviceSIProxy sip(nh,"caros_universalrobot");
//	caros::UrProxy urp(nh,"caros_universalrobot");
//
//	rw::math::Q q1(6, 3.15319, -1.4478, 1.37673, -1.4099, 1.57271, 2.60883);
//	rw::math::Q q2(6, 2.20541, -0.8386, 0.70459, -1.3793, 1.56800, 2.60880);
//	rw::math::Q q3(6, 1.40312, -1.4600, 1.68823, -1.7173, 1.57293, 2.60878);
//	rw::math::Q q4(6, 1.40169, -1.2605, 0.56003, -0.7888, 1.57509, 2.60431);
//
//	std::cout << "Program ID: "<< 5 << std::endl;
//
//	dsl::common::DataPrint dp1("/home/josl/Dropbox/forceexperiments/","test1.txt");
//	dsl::common::DataPrint dp2("/home/josl/Dropbox/forceexperiments/","test2.txt");
//	dsl::common::DataPrint dp3("/home/josl/Dropbox/forceexperiments/","test3.txt");
//	dsl::common::DataPrint dp4("/home/josl/Dropbox/forceexperiments/","test4.txt");
//
//	dp1.writeForceExtendedHeader();
//	dp2.writeForceExtendedHeader();
//	dp3.writeForceExtendedHeader();
//	dp4.writeForceExtendedHeader();
//
////	dslMove(q1,sip,urp,dp1);
////	dslMove(q2,sip,urp,dp2);
////	dslMove(q3,sip,urp,dp3);
////	dslMove(q4,sip,urp,dp4);
//
////	dslMoveDiff(q2,sip,urp,dp2);
////	dslMoveDiff(q3,sip,urp,dp3);
////	dslMoveDiff(q4,sip,urp,dp4);
////
//	dslServo(q1,sip,urp,dp1);
//	dslServo(q2,sip,urp,dp2);
//	dslServo(q3,sip,urp,dp3);
//	dslServo(q4,sip,urp,dp4);
//
//
//	std::cout << dsl::color::BOLDMAGENTA << "Program termination";
//	std::cout << dsl::color::DEFAULT << std::endl;
//
////		dsl::common::DataPrint dp("/home/josl/Dropbox/forceexperiments/","test2.txt");
//
//
//
//	//	int stepN = 100000;
//	//
//	//
//	//	rw::math::Q qc = sip.getQ();
//	//
//	//	for(int i = 0; i < stepN; i++)
//	//	{
//	//		rw::math::Q qk = q1+(q2-q1)/stepN*i;
//	//		dslMove(qk,sip,urp);
//	//	}
//


}

TestServo::~TestServo() {
}

} /* namespace dsl */


//dsl::common::DataPrint dp11("/home/josl/Dropbox/forceexperiments/","test1-1.txt");
//dsl::common::DataPrint dp21("/home/josl/Dropbox/forceexperiments/","test2-1.txt");
//dsl::common::DataPrint dp31("/home/josl/Dropbox/forceexperiments/","test3-1.txt");
//dsl::common::DataPrint dp41("/home/josl/Dropbox/forceexperiments/","test4-1.txt");
//dsl::common::DataPrint dp12("/home/josl/Dropbox/forceexperiments/","test1-2.txt");
//dsl::common::DataPrint dp22("/home/josl/Dropbox/forceexperiments/","test2-2.txt");
//dsl::common::DataPrint dp32("/home/josl/Dropbox/forceexperiments/","test3-2.txt");
//dsl::common::DataPrint dp42("/home/josl/Dropbox/forceexperiments/","test4-2.txt");
//dsl::common::DataPrint dp13("/home/josl/Dropbox/forceexperiments/","test1-3.txt");
//dsl::common::DataPrint dp23("/home/josl/Dropbox/forceexperiments/","test2-3.txt");
//dsl::common::DataPrint dp33("/home/josl/Dropbox/forceexperiments/","test3-3.txt");
//dsl::common::DataPrint dp43("/home/josl/Dropbox/forceexperiments/","test4-3.txt");
//dsl::common::DataPrint dp14("/home/josl/Dropbox/forceexperiments/","test1-4.txt");
//dsl::common::DataPrint dp24("/home/josl/Dropbox/forceexperiments/","test2-4.txt");
//dsl::common::DataPrint dp34("/home/josl/Dropbox/forceexperiments/","test3-4.txt");
//dsl::common::DataPrint dp44("/home/josl/Dropbox/forceexperiments/","test4-4.txt");
//dsl::common::DataPrint dp15("/home/josl/Dropbox/forceexperiments/","test1-5.txt");
//dsl::common::DataPrint dp25("/home/josl/Dropbox/forceexperiments/","test2-5.txt");
//dsl::common::DataPrint dp35("/home/josl/Dropbox/forceexperiments/","test3-5.txt");
//dsl::common::DataPrint dp45("/home/josl/Dropbox/forceexperiments/","test4-5.txt");
//dsl::common::DataPrint dp16("/home/josl/Dropbox/forceexperiments/","test1-6.txt");
//dsl::common::DataPrint dp26("/home/josl/Dropbox/forceexperiments/","test2-6.txt");
//dsl::common::DataPrint dp36("/home/josl/Dropbox/forceexperiments/","test3-6.txt");
//dsl::common::DataPrint dp46("/home/josl/Dropbox/forceexperiments/","test4-6.txt");
//dsl::common::DataPrint dp17("/home/josl/Dropbox/forceexperiments/","test1-7.txt");
//dsl::common::DataPrint dp27("/home/josl/Dropbox/forceexperiments/","test2-7.txt");
//dsl::common::DataPrint dp37("/home/josl/Dropbox/forceexperiments/","test3-7.txt");
//dsl::common::DataPrint dp47("/home/josl/Dropbox/forceexperiments/","test4-7.txt");
//dsl::common::DataPrint dp18("/home/josl/Dropbox/forceexperiments/","test1-8.txt");
//dsl::common::DataPrint dp28("/home/josl/Dropbox/forceexperiments/","test2-8.txt");
//dsl::common::DataPrint dp38("/home/josl/Dropbox/forceexperiments/","test3-8.txt");
//dsl::common::DataPrint dp48("/home/josl/Dropbox/forceexperiments/","test4-8.txt");
//dsl::common::DataPrint dp19("/home/josl/Dropbox/forceexperiments/","test1-9.txt");
//dsl::common::DataPrint dp29("/home/josl/Dropbox/forceexperiments/","test2-9.txt");
//dsl::common::DataPrint dp39("/home/josl/Dropbox/forceexperiments/","test3-9.txt");
//dsl::common::DataPrint dp49("/home/josl/Dropbox/forceexperiments/","test4-9.txt");
//dsl::common::DataPrint dp10("/home/josl/Dropbox/forceexperiments/","test1-0.txt");
//dsl::common::DataPrint dp20("/home/josl/Dropbox/forceexperiments/","test2-0.txt");
//dsl::common::DataPrint dp30("/home/josl/Dropbox/forceexperiments/","test3-0.txt");
//dsl::common::DataPrint dp40("/home/josl/Dropbox/forceexperiments/","test4-0.txt");
//dp11.writeForceHeader();
//dp21.writeForceHeader();
//dp31.writeForceHeader();
//dp41.writeForceHeader();
//dp12.writeForceHeader();
//dp22.writeForceHeader();
//dp32.writeForceHeader();
//dp42.writeForceHeader();
//dp13.writeForceHeader();
//dp23.writeForceHeader();
//dp33.writeForceHeader();
//dp43.writeForceHeader();
//dp14.writeForceHeader();
//dp24.writeForceHeader();
//dp34.writeForceHeader();
//dp44.writeForceHeader();
//dp15.writeForceHeader();
//dp25.writeForceHeader();
//dp35.writeForceHeader();
//dp45.writeForceHeader();
//dp16.writeForceHeader();
//dp26.writeForceHeader();
//dp36.writeForceHeader();
//dp46.writeForceHeader();
//dp17.writeForceHeader();
//dp27.writeForceHeader();
//dp37.writeForceHeader();
//dp47.writeForceHeader();
//dp18.writeForceHeader();
//dp28.writeForceHeader();
//dp38.writeForceHeader();
//dp48.writeForceHeader();
//dp19.writeForceHeader();
//dp29.writeForceHeader();
//dp39.writeForceHeader();
//dp49.writeForceHeader();
//dp10.writeForceHeader();
//dp20.writeForceHeader();
//dp30.writeForceHeader();
//dp40.writeForceHeader();
//
//
//dslServo(q1,sip,urp,dp11);
//dslServo(q2,sip,urp,dp21);
//dslServo(q3,sip,urp,dp31);
//dslServo(q4,sip,urp,dp41);
//dslServo(q1,sip,urp,dp12);
//dslServo(q2,sip,urp,dp22);
//dslServo(q3,sip,urp,dp32);
//dslServo(q4,sip,urp,dp42);
//dslServo(q1,sip,urp,dp13);
//dslServo(q2,sip,urp,dp23);
//dslServo(q3,sip,urp,dp33);
//dslServo(q4,sip,urp,dp43);
//dslServo(q1,sip,urp,dp14);
//dslServo(q2,sip,urp,dp24);
//dslServo(q3,sip,urp,dp34);
//dslServo(q4,sip,urp,dp44);
//dslServo(q1,sip,urp,dp15);
//dslServo(q2,sip,urp,dp25);
//dslServo(q3,sip,urp,dp35);
//dslServo(q4,sip,urp,dp45);
//dslServo(q1,sip,urp,dp16);
//dslServo(q2,sip,urp,dp26);
//dslServo(q3,sip,urp,dp36);
//dslServo(q4,sip,urp,dp46);
//dslServo(q1,sip,urp,dp17);
//dslServo(q2,sip,urp,dp27);
//dslServo(q3,sip,urp,dp37);
//dslServo(q4,sip,urp,dp47);
//dslServo(q1,sip,urp,dp18);
//dslServo(q2,sip,urp,dp28);
//dslServo(q3,sip,urp,dp38);
//dslServo(q4,sip,urp,dp48);
//dslServo(q1,sip,urp,dp19);
//dslServo(q2,sip,urp,dp29);
//dslServo(q3,sip,urp,dp39);
//dslServo(q4,sip,urp,dp49);
//dslServo(q1,sip,urp,dp10);
//dslServo(q2,sip,urp,dp20);
//dslServo(q3,sip,urp,dp30);
//dslServo(q4,sip,urp,dp40);
