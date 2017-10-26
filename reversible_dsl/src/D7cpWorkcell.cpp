/*
 * D7cpWorkcell.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: josl
 */

#include "D7cpWorkcell.h"

#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/calibration/xml/XmlCalibrationLoader.hpp>
#include "KukaIIWAPlanner.hpp"
//#include <tfi_planner/planners/PlannerInterface.hpp>

D7cpWorkcell::D7cpWorkcell()
{
	wcFilePath = "/home/josl/D7PC/model/full_scene.wc.xml";
	wcTrajoptFilePath = "/home/josl/D7PC/model/full_scene.or.xml";
	wc = rw::loaders::WorkCellLoader::Factory::load(wcFilePath);
	assert(!wc.isNull() && "Could not find workcell!");

	calibrationLoaded = rwlibs::calibration::XmlCalibrationLoader::load(wc, wc->getFilePath() + wc->getCalibrationFilename());
	assert(!calibrationLoaded.isNull() && "Could not load calibration information.");
	calibrationLoaded->apply();

	device = wc->findDevice("KukaIIWA");
	assert(!device.isNull() && "Could not find a device with the name KukaIIWA in the workcell!");

	plannerTnt = std::make_shared<KukaIIWAPlanner>(wc, "KukaIIWA");
//	std::shared_ptr<PlannerInterface> plannerTfi = std::make_shared<PlannerInterface>(wc, "KukaIIWA", wcTrajoptFilePath);
//	planner    = std::make_shared<Planner>(plannerTnt, plannerTfi, wc, device);
	planner    = std::make_shared<Planner>(plannerTnt, wc, device);

	std::shared_ptr<KukaIIWAPlanner> plannerTntRedundency = std::make_shared<KukaIIWAPlanner>(wc, "KukaIIWA", 2);
	plannerRedundency    = std::make_shared<Planner>(plannerTntRedundency, wc, device);


	toolFrame = findFrame("Tool.TCP");
	cameraFrame = findFrame("Camera3D3");
	fixtureOffsetFrame = findFrame("Fixture_offset");
	assemblyFrame = findFrame("Assembly");

	endFrame = std::shared_ptr<rw::kinematics::Frame>(device->getEnd());
	baseFrame = std::shared_ptr<rw::kinematics::Frame>(device->getBase());

	outershellFrame = findFrame("Part1_OuterShell");
	outershellGraspFrame = findFrame("Grasp_OuterShell");
	outershellDetachFrame = findFrame("Part1_Detach");
	outershellAssemblyFrame = findFrame("Assembly_OuterShell");
	outershellAssemblyRetractedFrame = findFrame("Assembly_OuterShell_Retracted");
	outershellFixedPickTcpFrame = findFrame("Pick_OuterShell_TCP");
	outershellFixedPickRetractedTcpFrame= findFrame("Pick_OuterShell_Retracted_TCP");
	outershellAltColModelFrame = findFrame("Part1_OuterShell_Alternative_Collision_Model");
	outershellAltDetachFrame = findFrame("Part1Alt_Detach");
	outershellInnerRing = findFrame("Assembly_OuterShell_Inner_Ring");



	innerStructureFrame = findFrame("Part2_InnerStructure");
	innerStructureGraspFrame = findFrame("Grasp_InnerStructure");
	innerStructureDetachFrame = findFrame("Part2_Detach");
	innerStructureAssemblyFrame = findFrame("Assembly_InnerStructure");
	innerStructureAssemblyRetractedFrame = findFrame("Assembly_InnerStructure_Retracted");
	innerStructureFixedPickTcpFrame = findFrame("Pick_InnerStructure_TCP");
	innerStructureFixedPickRetractedTcpFrame= findFrame("Pick_InnerStructure_Retracted_TCP");
	innerStructureAltColModelFrame = findFrame("Part2_InnerStructure_Alternative_Collision_Model");
	innerStructureAltDetachFrame = findFrame("Part2Alt_Detach");
	innerStructureFemaleAssemblyFrame = findFrame("Assembly_InnerStructure_female");
	innerStructureTipInGripper = findFrame("InnerStructure_Grasped_Tip");

	thermoelementFrame = findFrame("Part3_Thermoelement");
	thermoelementGraspFrame = findFrame("Grasp_Thermoelement");
	thermoelementDetachFrame = findFrame("Part3_Detach");
	thermoelementAssemblyFrame = findFrame("Assembly_Thermoelement");
	thermoelementAssemblyRetractedFrame = findFrame("Assembly_Thermoelement_Retracted");

	springFrame = findFrame("Part4_Spring");
	springGraspFrame = findFrame("Grasp_Spring");
	springDetachFrame = findFrame("Part4_Detach");
	springAssemblyFrame = findFrame("Assembly_Spring");
	springAssemblyRetractedFrame = findFrame("Assembly_Spring_Retracted");

	pinFrame = findFrame("Part5_Pin");
	pinGraspFrame = findFrame("Grasp_Pin");
	pinDetachFrame = findFrame("Part5_Detach");
	pinAssemblyFrame = findFrame("Assembly_Pin");
	pinAssemblyRetractedFrame = findFrame("Assembly_Pin_Retracted");
	pinInsertionFrame = findFrame("Insert_Pin");

	screwPartFrame = findFrame("Part6_ScrewPart");
	screwPartGraspFrame = findFrame("Grasp_ScrewPart");
	screwPartDetachFrame = findFrame("Part6_Detach");
	screwPartAssemblyFrame = findFrame("Assembly_ScrewPart");
	screwPartAssemblyRotatedFrame = findFrame("Assembly_ScrewPart_Rotated");
	screwPartAssemblyRetractedFrame = findFrame("Assembly_ScrewPart_Retracted");
	screwPartAssemblyRetractedRotatedFrame = findFrame("Assembly_ScrewPart_Retracted_Rotated");
	screwPartFixedPickTcpFrame = findFrame("Pick_ScrewPart_TCP");
	screwPartFixedPickRetractedTcpFrame= findFrame("Pick_ScrewPart_Retracted_TCP");

	qInspectOuterShell = findQ("InspectOuterShell");
	qInspectInnerStructure = findQ("InspectInnerStructure");
	qInspectScrewPart = findQ("InspectScrewPart");
	qInspectPin = findQ("InspectPin");
	qDispenser = findQ("Dispenser");

	qSpringPickInit = findQ("SpringPickInit");
	qSpringPickPath1 = findQ("SpringPickPath1");
	qSpringPickPath2 = findQ("SpringPickPath2");

	qThermoelementPickInit = findQ("ThermoelementPickInit");

	//qThermoelementPath1 = findQ("ThermoelementPath1");
	qThermoelementPath2 = findQ("ThermoelementPath2");
	qThermoelementEnd = findQ("ThermoelementEnd");

	for (unsigned int i = 0; i < Ni; i++)
		for (unsigned int j = 0; j < Nj; j++)
		{
			std::stringstream name;
			name << "PinHole" << i+1 << j+1;
			std::stringstream nameAbove;
			nameAbove << "PinHoleAbove" << i+1 << j+1;
			qPinHole[i][j] = findQ(name.str());
			qPinHoleAbove[i][j] = findQ(nameAbove.str());
		}

	qZero = rw::math::Q(7, 0, 0, 0, 0, 0, 0, 0);
	qAssemblySpringIntermediet =rw::math::Q(7,0.4804444006859994, 0.6122837576018999, -0.713123939555106, -1.9688465446742534, 0.31696855936576124, 0.692562904497, 2.7660277777539255);
	qPickRetractedInnerstrucutre = rw::math::Q(7, -1.3801035198970466, 0.942743905773491, -0.05031181342234474, -1.3298346698268182, 0.14315645602006655, 1.6515006139700836, -1.4737607768425558);

	pathplannerAnyFeederInit = findFrame("AnyFeeder_Pathplanner_Init");

	converter = std::make_shared<Converter>(wc, device, toolFrame);

}

D7cpWorkcell::~D7cpWorkcell()
{
}



rw::kinematics::State D7cpWorkcell::getDefaultState()
{
	return wc->getDefaultState();
}

void D7cpWorkcell::setQ(const rw::math::Q& q, rw::kinematics::State& state) const
{
	device->setQ(q, state);
}

void D7cpWorkcell::attachTo(std::shared_ptr<rw::kinematics::Frame> child, std::shared_ptr<rw::kinematics::Frame> parent, rw::kinematics::State& state) const
{
	child.get()->attachTo(parent.get(), state);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PRIVATE CONSTRUCTION FUNCTIONS

std::shared_ptr<rw::kinematics::Frame> D7cpWorkcell::findFrame(std::string frameName) const
{
	assert(!wc.isNull() && "Could not find workcell!");
	std::shared_ptr<rw::kinematics::Frame> frame(wc->findFrame(frameName));
	if(frame == NULL)
		std::cerr << "Could not find the frame " << frameName << std::endl;
	return frame;
}

rw::math::Q D7cpWorkcell::findQ(std::string qName) const
{
	assert(!device.isNull() && "Could not find device");
	rw::math::Q q = device->getPropertyMap().get<rw::math::Q>(qName);
	if(q.size() != 7)
		std::cerr << "Could not find q with name or size 7" << qName << std::endl;
	return q;
}

