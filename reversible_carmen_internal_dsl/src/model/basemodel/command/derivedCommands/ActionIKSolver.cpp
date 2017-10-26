/*
 * SceeneIKSolver.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: jpb
 */

#include <../src/model/basemodel/command/derivedCommands/ActionIKSolver.hpp>

#include <cassert>
#include <rw/loaders/WorkCellLoader.hpp>
#include <iostream>
#include <fstream>

namespace dsl {

using namespace rw::trajectory;


ActionIKSolver::ActionIKSolver(std::string wcStig, std::string deviceName, std::string toolFrame, std::string refFrame)
{
	_wc =  rw::loaders::WorkCellLoader::Factory::load(wcStig);
	RW_ASSERT(_wc != NULL);

	_device = _wc->findDevice(deviceName);
	_state = _wc->getDefaultState();
	_tool = _wc->findFrame(toolFrame);
	_ref = _wc->findFrame(refFrame);

	if(_device == NULL)
		std::cout << "Failed to find device with name: " << deviceName << std::endl;

	if(_tool == NULL)
		std::cout << "Failed to find toolFrame with name: " << toolFrame << std::endl;

	RW_ASSERT(_device != NULL);
	RW_ASSERT(_tool != NULL);

	_invKin = new rw::invkin::JacobianIKSolver(_device,_tool,_state);
	_invKin->setSolverType(_invKin->SVD);
	////    invKin->setCheckJointLimits(true);
	////    invKin->setClampToBounds(true);
	////    invKin->setEnableInterpolation(true);
	////    invKin->setInterpolatorStep();
	////    invKin->setMaxError(0.000001);
	////    invKin->setMaxIterations();
//	std::cout << "test 4" << std::endl;
}

void ActionIKSolver::setInitialQ(rw::math::Q q)
{
	_device->setQ(q, _state);
}

void ActionIKSolver::setOffsetTransform(rw::math::Transform3D<> offset)
{
	_offsetTransform = offset;
}

void ActionIKSolver::setOffsetTransformIdentity()
{
	rw::math::Transform3D<> transformIdentity;
	_offsetTransform = transformIdentity;
}




rw::trajectory::TimedQPath ActionIKSolver::solve(std::string trajectoryFileName)
{
	std::cout << "Solving: " << trajectoryFileName << std::endl;
	std::ifstream readFile;
	readFile.open(trajectoryFileName);

//	std::cout << "test 6.6" << std::endl;

	if(readFile.is_open() == false)
	{
		std::cerr << "Failed to load: " << trajectoryFileName << std::endl;
		exit(-1);
	}

//	std::cout << "test 6.61" << std::endl;


	rw::trajectory::Path<Timed<rw::math::Transform3D<> > > PathTimedTransform3D;

//	std::cout << "test 6.62" << std::endl;

	char output[1000];
	if (readFile.is_open() )
	{
		while (!readFile.eof()) {
//			std::cout << "test 6.63" << std::endl;

			double dimentions[7];
			for(int dim=0; dim< 7;dim++)
			{
//				std::cout << "test 6.64" << std::endl;
				readFile >> output;
//				std::cout << "test 6.65" << std::endl;
				dimentions[dim] = atof(output);
//				std::cout << "test 6.66" << std::endl;
			}
//			std::cout << "test 6.67" << std::endl;

			rw::math::Transform3D<> transform = rw::math::Transform3D<>(rw::math::Vector3D<>(dimentions[0],dimentions[1],dimentions[2]),rw::math::RPY<>(dimentions[3],dimentions[4],dimentions[5]).toRotation3D());
			rw::trajectory::Timed<rw::math::Transform3D<> > timedT = rw::trajectory::Timed<rw::math::Transform3D<> >(dimentions[6],transform);
			PathTimedTransform3D.push_back(timedT);
		}
	}
//		std::cout << "test 6.7" << std::endl;
	 PathTimedTransform3D.pop_back();

//for(Timed<Transform3D<> > tran : PathTimedTransform3D)
//{
//	sim::utility::print(tran.getValue());
//}
//		std::cout << "test 6.8" << std::endl;
	return solve(PathTimedTransform3D);
}

rw::trajectory::TimedQPath ActionIKSolver::solve(rw::trajectory::Path<rw::trajectory::Timed<rw::math::Transform3D<> > > T3DPath)
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	std::cout << " Solving transform3Dpath" << std::endl;
	// path(timed(Q))
	rw::trajectory::TimedQPath tQpath;
	for(rw::trajectory::Timed<rw::math::Transform3D<> > femaleTCPTmaleTCPtimed : T3DPath)
	{

		rw::math::Transform3D<> femaleTCPTmaleTCP = femaleTCPTmaleTCPtimed.getValue();
		rw::math::Transform3D<> worldTPalette = _wc->findFrame("PaletteRobot")->getTransform(_state);
		rw::math::Transform3D<> palrobTbase = _wc->findFrame("Arm1Base")->getTransform(_state);

		rw::math::Transform3D<> worldTref = _ref->wTf(_state);
		rw::math::Transform3D<> worldToffset = worldTref * _offsetTransform;
//		rw::math::Transform3D<> baseTend = inverse(worldTPalette * palrobTbase) * worldTref * femaleTCPTmaleTCP;
		rw::math::Transform3D<> baseTend = inverse(worldTPalette * palrobTbase) * worldToffset * femaleTCPTmaleTCP;

		// END IN THIS IS THE YDERSTE CHILD FRAME OG IKKE DET SAMME SOM DU FÅR NÅR DU KALDER device->getEnd()->getName()
		std::vector<rw::math::Q> qSolutions = _invKin->solve(baseTend , _state);
		if( qSolutions.size()>0 )
		{
			tQpath.push_back(  TimedQ(femaleTCPTmaleTCPtimed.getTime() , qSolutions[0]) );
			_device->setQ(qSolutions[0],_state);
		}
		else
		{
			std::cerr << "Error no solution found for the inverse kinematics solver" << std::endl;
		}

	}
	std::cout << " Found trajectory of size: " << tQpath.size() << std::endl;
	return tQpath;
}

} /* namespace dsl */
