/*
 * Anyfeeder.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: josl
 */

#include "Anyfeeder.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <anyfeeder_driver/DispenseAnyFeederAction.h>
#include <anyfeeder_driver/FeedBackwardAnyFeederAction.h>
#include <anyfeeder_driver/FeedFlipBackwardAnyFeederAction.h>
#include <anyfeeder_driver/FeedFlipFowardAnyFeederAction.h>
#include <anyfeeder_driver/FeedFowardAnyFeederAction.h>
#include <anyfeeder_driver/FlipAnyFeederAction.h>
#include <anyfeeder_driver/HeavyDispenseAnyFeederAction.h>
#include <anyfeeder_driver/InitAnyFeederAction.h>
#include <anyfeeder_driver/PurgeAnyFeederAction.h>


Anyfeeder::Anyfeeder() :
	feed_counter(0)
{
}

Anyfeeder::~Anyfeeder()
{
}


bool Anyfeeder::shakeFeeder()
{
	feed_counter += 1;
	if(feed_counter % 4 == 0)
		return feed_backward_any_feeder_action();
	return feed_foward_any_feeder_action();

}

bool Anyfeeder::dispense_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::DispenseAnyFeederAction> client("dispense_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::DispenseAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::feed_backward_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::FeedBackwardAnyFeederAction> client("feed_backward_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::FeedBackwardAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;

}

bool Anyfeeder::feed_flip_backward_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::FeedFlipBackwardAnyFeederAction> client("feed_flip_backward_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::FeedFlipBackwardAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::feed_flip_foward_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::FeedFlipFowardAnyFeederAction> client("feed_flip_foward_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::FeedFlipFowardAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::feed_foward_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::FeedFowardAnyFeederAction> client("feed_foward_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::FeedFowardAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::flip_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::FlipAnyFeederAction> client("flip_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::FlipAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::heavy_dispense_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::HeavyDispenseAnyFeederAction> client("heavy_dispense_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::HeavyDispenseAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::start_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::InitAnyFeederAction> client("start_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::InitAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::init_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::InitAnyFeederAction> client("init_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::InitAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool Anyfeeder::purge_any_feeder_action()
{
	actionlib::SimpleActionClient<anyfeeder_driver::PurgeAnyFeederAction> client("purge_any_feeder_action", true);
	client.waitForServer();
	anyfeeder_driver::PurgeAnyFeederGoal goal;
	goal.execute = true;
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Anyfeeder failed in %s ",__PRETTY_FUNCTION__);
		return false;
	}
	ros::Duration(2.5).sleep();
	return true;
}



/*



/dispense_any_feeder_action/cancel
/feed_backward_any_feeder_action/cancel
/feed_flip_backward_any_feeder_action/cancel
/feed_flip_foward_any_feeder_action/cancel
/feed_foward_any_feeder_action/cancel
/flip_any_feeder_action/cancel
/heavy_dispense_any_feeder_action/cancel
/init_any_feeder_action/cancel
/purge_any_feeder_action/cancel



 */

