/*
 * MockAutopilotInterface.h
 *
 *  Created on: Aug 24, 2018
 *      Author: gmoreno
 */

#ifndef MOCKAUTOPILOTINTERFACE_H_
#define MOCKAUTOPILOTINTERFACE_H_

#include "autopilot_interface.h"
#include <chrono>


/**
 * Mock implementation of Autopilot_Interface
 */
class Mock_Autopilot_Interface: public Autopilot_Interface {
	bool isArmed;
	bool isFlying;
	std::chrono::high_resolution_clock::time_point start_time;

public:
	Mock_Autopilot_Interface(std::shared_ptr<Port> port_);
	virtual ~Mock_Autopilot_Interface();

	virtual void dumpCurrentMavlinkMessages();
	virtual void arm();
	virtual void disarm();
	virtual void set_manual_control(double roll, double pitch, double yaw, double throttle);
	virtual void set_posctl_mode();
	virtual void set_manual_mode();
	virtual void start();
	virtual void stop();
	virtual void click_button(unsigned button);
	virtual void set_reboot_period(int32_t period);
	virtual void land();
	virtual void hover();
	virtual void takeoff();
	virtual int get_battery_remaining();
	virtual bool is_flying();
	virtual std::string get_position();
	virtual bool is_home_set();
	virtual void setTicksToReset(int ticks);
	virtual void request_autopilot_capabilities(float param1, float param2);

};

#endif /* MOCKAUTOPILOTINTERFACE_H_ */
