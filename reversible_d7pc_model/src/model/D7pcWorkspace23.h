/*
 * D7pcWorkspace23.h
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_GRAPHVIZ_D7PCWORKSPACE23_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_GRAPHVIZ_D7PCWORKSPACE23_H_

#include <memory.h>
#include <rw/common/Ptr.hpp>

#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/calibration/WorkCellCalibration.hpp>
#include "Planner.h"
#include "Converter.h"

namespace model {

class D7pcWorkspace23 {
public:
	D7pcWorkspace23();
	virtual ~D7pcWorkspace23();

	void init();
	void initEsentials();
	void initVariables();

	rw::kinematics::State getDefaultState();
    void setQ(const rw::math::Q& q, rw::kinematics::State& state) const;
    void attachTo(std::shared_ptr<rw::kinematics::Frame> child, std::shared_ptr<rw::kinematics::Frame> parent, rw::kinematics::State& state) const;
//
//private:
//	std::shared_ptr<rw::kinematics::Frame> findFrame(std::string frameName) const;
//	rw::math::Q findQ(std::string qName) const;


public:
	std::string wcFilePath;
	std::string wcTrajoptFilePath;
	rw::models::WorkCell::Ptr wc;
//	rwlibs::calibration::WorkCellCalibration::Ptr calibrationLoaded;
	rw::models::Device::Ptr device;
//	std::shared_ptr<KukaIIWAPlanner> plannerTnt;
//	std::shared_ptr<Planner> planner;
//	std::shared_ptr<Planner> plannerRedundency;
//	std::shared_ptr<Converter> converter;
//
//
//
//
//
//	std::shared_ptr<rw::kinematics::Frame> toolFrame;
//	std::shared_ptr<rw::kinematics::Frame> cameraFrame;
//	std::shared_ptr<rw::kinematics::Frame> fixtureOffsetFrame;
//	std::shared_ptr<rw::kinematics::Frame> assemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> endFrame;
//	std::shared_ptr<rw::kinematics::Frame> baseFrame;
//
//
//	std::shared_ptr<rw::kinematics::Frame> outershellFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellGraspFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellAssemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellAssemblyRetractedFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellFixedPickTcpFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellFixedPickRetractedTcpFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellAltColModelFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellAltDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> outershellInnerRing;
//
//	std::shared_ptr<rw::kinematics::Frame> innerStructureFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureFemaleAssemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureGraspFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureAssemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureAssemblyRetractedFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureFixedPickTcpFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureFixedPickRetractedTcpFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureAltColModelFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureAltDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> innerStructureTipInGripper;
//
//	std::shared_ptr<rw::kinematics::Frame> thermoelementFrame;
//	std::shared_ptr<rw::kinematics::Frame> thermoelementGraspFrame;
//	std::shared_ptr<rw::kinematics::Frame> thermoelementDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> thermoelementAssemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> thermoelementAssemblyRetractedFrame;
//
//	std::shared_ptr<rw::kinematics::Frame> springFrame;
//	std::shared_ptr<rw::kinematics::Frame> springGraspFrame;
//	std::shared_ptr<rw::kinematics::Frame> springDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> springAssemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> springAssemblyRetractedFrame;
//
//	std::shared_ptr<rw::kinematics::Frame> pinFrame;
//	std::shared_ptr<rw::kinematics::Frame> pinInsertionFrame;
//	std::shared_ptr<rw::kinematics::Frame> pinGraspFrame;
//	std::shared_ptr<rw::kinematics::Frame> pinDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> pinAssemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> pinAssemblyRetractedFrame;
//
//	std::shared_ptr<rw::kinematics::Frame> screwPartFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartGraspFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartDetachFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartAssemblyFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartAssemblyRotatedFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartAssemblyRetractedFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartAssemblyRetractedRotatedFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartFixedPickTcpFrame;
//	std::shared_ptr<rw::kinematics::Frame> screwPartFixedPickRetractedTcpFrame;
//
////	std::shared_ptr<D7cpObject> outershell;
////	std::shared_ptr<D7cpObject> innerstructure;
////	std::shared_ptr<D7cpObject> thermoelement;
////	std::shared_ptr<D7cpObject> spring;
////	std::shared_ptr<D7cpObject> pin;
////	std::shared_ptr<D7cpObject> screwpart;
//
//
//	rw::math::Q qInspectOuterShell;
//	rw::math::Q qInspectInnerStructure;
//	rw::math::Q qInspectScrewPart;
//	rw::math::Q qInspectPin;
//	rw::math::Q qDispenser;
//
//	rw::math::Q qSpringPickInit;
//	rw::math::Q qSpringPickPath1;
//	rw::math::Q qSpringPickPath2;
//
//	rw::math::Q qThermoelementPickInit;
//	//rw::math::Q qThermoelementPath1;
//	rw::math::Q qThermoelementPath2;
//	rw::math::Q qThermoelementEnd;
//
//	static const unsigned int Ni = 3;
//	static const unsigned int Nj = 6;
//    rw::math::Q qPinHole[Ni][Nj];
//    rw::math::Q qPinHoleAbove[Ni][Nj];
//
//
//	rw::math::Q qZero;
//	rw::math::Q qAssemblySpringIntermediet;
//	rw::math::Q qPickRetractedInnerstrucutre;
//
//	std::shared_ptr<rw::kinematics::Frame> pathplannerAnyFeederInit;
};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_GRAPHVIZ_D7PCWORKSPACE23_H_ */
