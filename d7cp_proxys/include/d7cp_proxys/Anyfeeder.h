/*
 * Anyfeeder.h
 *
 *  Created on: Jan 31, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_D7CP_PROXYS_SRC_ANYFEEDER_H_
#define D7PC_ROS_D7CP_PROXYS_SRC_ANYFEEDER_H_

class Anyfeeder
{

public:
	Anyfeeder();
	virtual ~Anyfeeder();
	bool shakeFeeder();

	bool dispense_any_feeder_action();
	bool feed_backward_any_feeder_action();
	bool feed_flip_backward_any_feeder_action();
	bool feed_flip_foward_any_feeder_action();
	bool feed_foward_any_feeder_action();
	bool flip_any_feeder_action();
	bool heavy_dispense_any_feeder_action();
	bool start_any_feeder_action();
	bool init_any_feeder_action();
	bool purge_any_feeder_action();

private:
	int feed_counter;


};

#endif /* D7PC_ROS_D7CP_PROXYS_SRC_ANYFEEDER_H_ */
