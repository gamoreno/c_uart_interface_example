/*
 * stdio_control.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: gmoreno
 */

#include <unistd.h>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include "stdio_control.h"
#include "utils.h"

#include <common/mavlink.h>
#include <vector>
#include <random>

using namespace std;

#define TIMED_ATTACK 10
#define MAGIC_NUN_ATTACK 11
#define MAGIC_NUM_RANGE 10
#define MAGIC_NUMBER 3

void attack(Autopilot_Interface &api) {

	std::vector<int> numbers;
	for (int i = 0; i < MAGIC_NUM_RANGE; i++) {
		numbers.push_back(i);
	}

	std::default_random_engine generator;

	int attempt = 0;
	do {
		if (numbers.empty()) {
			cerr << "Attack failed" << endl;
			return;
		}
		std::uniform_int_distribution<> uniform(0, numbers.size() - 1);
		int index = uniform(generator);
		int number = numbers.at(index);
		numbers.erase(numbers.begin() + index);

		cerr << "Attack attempt " << ++attempt << ": " << number << endl;
		api.request_autopilot_capabilities(MAGIC_NUN_ATTACK, number);
		usleep(500000);
		if (api.current_messages.time_stamps.version) {
			cerr << "Got OS version: " << api.current_messages.version.os_sw_version << endl;
		}
	} while(api.current_messages.time_stamps.version == 0 || api.current_messages.version.os_sw_version != 0xdead);
	cerr << "Attack succeeded" << endl;
}

void stdio_control(Autopilot_Interface &api) {
	api.set_manual_control(0.0, 0.0, 0.0, 0.0);

	bool shutdown = false;
	while (!shutdown) {
		std::string line;
		while (std::getline(cin, line)) {
			//cerr << "got input [" << line << "]" << endl;
			auto cmd = splitString(line, " ");
			if (cmd.empty()) {
				continue;
			}
			if (cmd[0] == "tyrp" && cmd.size() >= 5) {
				api.set_posctl_mode();
				double thrust = (atof(cmd[1].c_str()) + 1.0) / 2.0;
				double yaw = atof(cmd[2].c_str());
				double roll = atof(cmd[3].c_str());
				double pitch = atof(cmd[4].c_str());
				//cerr << "got tyrp cmd " << roll << ' ' << pitch << ' ' << yaw << ' ' << thrust << endl;
				api.set_manual_control(roll, pitch, yaw, thrust);
				if (cmd.size() >= 6) {
					int steps = atoi(cmd[5].c_str());
					api.setTicksToReset(steps * api.WRITE_HZ / 20.0); // drone.js uses 20Hz
					//cerr << "set ticks" << endl;
				}
			} else if (cmd[0] == "ready?") {
				cout << "ready=" << ((api.is_home_set()) ? "true" : "false") << endl;
			} else if (cmd[0] == "hover") {
				api.hover();
			} else if (cmd[0] == "land") {
				api.land();
			} else if (cmd[0] == "takeoff") {
				api.takeoff();
			} else if (cmd[0] == "battery?") {
				cout << "battery=" << api.get_battery_remaining() << endl;
			} else if (cmd[0] == "flying?") {
				cout << "flying=" << ((api.is_flying()) ? "true" : "false") << endl;
			} else if (cmd[0] == "location?") {
				cout << "location=" << api.get_position() << endl;
			} else if (cmd[0] == "reboot_switch") {
				api.click_button(1);
			} else if (cmd[0] == "snapshot_switch") {
				api.click_button(2);
			} else if (cmd[0] == "reboot_period" and cmd.size() == 2) {
				int32_t period = atoi(cmd[1].c_str());
				//cerr << "setting reboot period to " << std::dec
				//		<< period << endl;
				api.set_reboot_period(period);
			} else if (cmd[0] == "shutdown") {
				shutdown = true;
				break;
			} else if (cmd[0] == "attack") {
				attack(api);
			} else if (cmd[0] == "manual") {
				api.set_manual_mode();
				api.arm();
			}
		}
	}

	// shutdown
	cerr << "shutting down..." << endl;
	api.disarm();
}
