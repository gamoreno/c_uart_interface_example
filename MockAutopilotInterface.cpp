/*
 * MockAutopilotInterface.cpp
 *
 *  Created on: Aug 24, 2018
 *      Author: gmoreno
 */

#include "MockAutopilotInterface.h"
#include <iostream>

using namespace std;

Mock_Autopilot_Interface::Mock_Autopilot_Interface(std::shared_ptr<Port> port_)
	: Autopilot_Interface(port_), isArmed(false), isFlying(false)
{
}

Mock_Autopilot_Interface::~Mock_Autopilot_Interface() {
	// TODO Auto-generated destructor stub
}

void Mock_Autopilot_Interface::dumpCurrentMavlinkMessages() {
}

void Mock_Autopilot_Interface::arm() {
	if (is_home_set()) {
		isArmed = true;
		//cerr << "status: armed" << endl;
	}
}

void Mock_Autopilot_Interface::disarm() {
	isArmed = false;
}

void Mock_Autopilot_Interface::set_manual_control(double roll, double pitch,
		double yaw, double throttle) {
}

void Mock_Autopilot_Interface::set_posctl_mode() {
}

void Mock_Autopilot_Interface::set_manual_mode() {
}

void Mock_Autopilot_Interface::start() {
	start_time = std::chrono::high_resolution_clock::now();
}

void Mock_Autopilot_Interface::stop() {
}

void Mock_Autopilot_Interface::click_button(unsigned button) {
}

void Mock_Autopilot_Interface::set_reboot_period(int32_t period) {
}

void Mock_Autopilot_Interface::land() {
	isFlying = false;

	// auto disarm
	disarm();
}

void Mock_Autopilot_Interface::hover() {
}

void Mock_Autopilot_Interface::takeoff() {

	// auto arm
	arm();
	if (isArmed) {
		isFlying = true;
	}
}

int Mock_Autopilot_Interface::get_battery_remaining() {
	return 90;
}

bool Mock_Autopilot_Interface::is_flying() {
	return isFlying;
}

std::string Mock_Autopilot_Interface::get_position() {
	if (isFlying) {
		return std::string("0,0,-2,0,0,0,0,0,0");
	} else {
		return std::string("0,0,0,0,0,0,0,0,0");
	}
}

bool Mock_Autopilot_Interface::is_home_set() {
	auto delta = std::chrono::high_resolution_clock::now() - start_time;
	int seconds_since_start = std::chrono::duration_cast<std::chrono::seconds>(delta).count();
	return (seconds_since_start >= 3);
}

void Mock_Autopilot_Interface::setTicksToReset(int ticks) {
}

void Mock_Autopilot_Interface::request_autopilot_capabilities(float param1,
		float param2) {
}
