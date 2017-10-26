/*
 * ManualErrorInjecter.hpp
 *
 *  Created on: Jan 26, 2015
 *      Author: josl
 */

#ifndef MANUALERRORINJECTER_HPP_
#define MANUALERRORINJECTER_HPP_

#include <ros/ros.h>
#include <rw/common/Ptr.hpp>

#include <../src/model/basemodel/controller/messages/MessagesError.hpp>
#include <../src/model/basemodel/controller/messages/MessagesContainer.hpp>

namespace dsl
{

class ManualErrorInjecter
{

public:
	ManualErrorInjecter(dsl::MessagesContainer * container);
	virtual ~ManualErrorInjecter();

	void run();
	void stop();
	void loop();

private:
	void injectError();

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

private:
	bool _isRunning;
	dsl::MessagesError * _msg;

	dsl::MessagesContainer * _container;
//	ros::Rate _loopRate;

};

} /* namespace dsl */

#endif /* MANUALERRORINJECTER_HPP_ */
