/*
 * dslMain.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: josl
 */

#include <ros/ros.h>


#include <boost/thread.hpp>
#include <rw/RobWork.hpp>
#include <rw/common/Ptr.hpp>

#include <RobotGeneralInterfaceProxy.hpp>
#include <../src/model/basemodel/command/derivedCommands/VisionAct.hpp>
#include <../src/programs/getURJointconfiguration.hpp>
#include <../src/programs/TestLanguage.hpp>
#include <../src/programs/TestLanguageXML.hpp>
#include <../src/programs/TestPrint.hpp>
#include <../src/programs/TestServo.hpp>
#include <../src/programs/TestIntelligentMove.hpp>
#include <../src/programs/TestNewURInterface.hpp>
#include <../src/programs/TestJoy.hpp>
#include <../src/programs/TestURReconnect.hpp>
#include <../src/programs/TestNikolajInterface.hpp>
#include <../src/programs/TestIntActionDataExtraction.hpp>
#include <../src/programs/TestIntDataExtraction2.hpp>

class Main
{

public:
	bool _runSpinThread;
	boost::thread _spinThread;

	void thread_func()
	{
		while (_runSpinThread)
		{
			ros::spinOnce();
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}

	void start_spin_thread()
	{
		_runSpinThread = true;
		_spinThread = boost::thread(&Main::thread_func, this);
	}

	void stop_spin_thread()
	{
		_runSpinThread = false;
		_spinThread.join();
	}


	void program_selection(int argc, char **argv)
	{

		std::vector<std::string> args(argv, argv+argc);
		std::cout << "Program inputs: " << std::endl;
		for(unsigned int i = 0; i < args.size(); i++)
			std::cout << i << ": " << args[i] << std::endl;


		for (size_t i = 1; i < args.size(); ++i)
		{
			if (args[i] == "-va")
				dsl::VisionAct act(std::shared_ptr<RobotGeneralInterfaceProxy>(new RobotGeneralInterfaceProxy(rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle), "ur5")));
			else if (args[i] == "-tl")
				dsl::TestLanguage tl(args);
			else if (args[i] == "-urr")
				TestURReconnect urr;
			else if (args[i] == "-tj")
				dsl::TestJoy tj;
			else if (args[i] == "-xml")
				dsl::TestLanguageXML xml;
			else if (args[i] == "-servo")
				dsl::TestServo srv;
			else if (args[i] == "-imove")
				dsl::TestIntelligentMove iMove;
			else if (args[i] == "-testprint")
				dsl::TestPrint tp;
			else if (args[i] == "-testur")
				TestNewURInterface urt;
			else if (args[i] == "-getq")
				getURJointconfiguration urq;
			else if (args[i] == "-nikolaj")
				TestNikolajInterface tni;
			else if (args[i] == "-tiad")
				dsl::intelligentmove::TestIntActionDataExtraction tdi;
			else if (args[i] == "-tiad2")
				dsl::intelligentmove::TestIntDataExtraction2 tdi;
			else if (args[i] == "-h" || true)
			{
				std::cout << "Usage is "
						"\n -h  : help "
						"\n -va : visual action"
						"\n -tl : test language"
						"\n -tj : test joystik"
						"\n -xml : visual action"
						"\n -servo : servo"
						"\n -imove intelligentMove test"
						"\n -testprint : printTest"
						"\n -testur : ur interface"
						"\n -getq : getURJointconfiguration"
						"\n -nikolaj : TestNikolajInterface"
						"\n -tiad TestIntelligentActionDataExtraction"
						"\n -tiad2 TestIntelligentActionDataExtraction"
						"\n" << std::endl;
				exit(-0);
			}
		}
	}
};


int main(int argc, char **argv) {

	ros::init(argc, argv, "sdu_dsl");
	ros::start();

	Main aci;
	aci.start_spin_thread();
	aci.program_selection(argc, argv);
	aci.stop_spin_thread();
	return 0;
}
