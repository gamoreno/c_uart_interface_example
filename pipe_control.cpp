/*
 * pipe_control.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: gmoreno
 */

#include "pipe_control.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <fstream>

using namespace std;


const char* PIPE_NAME = "/tmp/manual_input_pipe";


void remoteCommands(Autopilot_Interface &api)
{

	if (mknod(PIPE_NAME, S_IFIFO | 0600, 0) != 0) {
		if (errno != EEXIST) {
			cerr << "Error creating pipe " << PIPE_NAME << ": " << strerror(errno) << endl;
			return;
		}
	}

	api.set_manual_control(0.0, 0.0, 0.0, 0.0);

	api.arm();
	sleep(2);

	api.set_posctl_mode();
	sleep(2);
	api.set_posctl_mode();


	bool shutdown = false;
	while (!shutdown) {
		std::ifstream pipe(PIPE_NAME);
		if (pipe) {
			std::string line;
			while (std::getline(pipe, line)) {
				cout << "got input [" << line << "]" << endl;
		    	    std::istringstream ss(line);
		    	    char cmd;
		    	    ss >> cmd;
		    	    //cout << "got cmd [" << cmd << "]" << endl;
		    	    if (cmd == 'i') {
		    	    	double roll;
		    	    	double pitch;
		    	    	double yaw;
		    	    	double thrust;
		    	    	if (!(ss >> roll >> pitch >> yaw >> thrust)) {
		    	    		cout << "cmd error" << endl;
					break;
		    	    	} else {
		    	    		//cout << "got " << roll << ' ' << pitch << ' ' << yaw << ' ' << thrust << endl;
		    	    		api.set_manual_control(roll, pitch, yaw, thrust);
		    	    	}
		    	    } else if (cmd == 'r') {
				api.click_button(1);
#if 0
				string arg;
				getline(ss, arg);
				if (arg.length() > 0 && arg[0] == ' ') {
					arg = arg.substr(1);
				}
				if (arg == "off") {
					cout << "turning reboots off" << endl;
					api.set_buttons(0);
				} else if (arg == "on") {
					cout << "turning reboots on" << endl;
					api.set_buttons(1 << (REBOOT_SWITCH_CHANNEL - 5));
				} else if (arg == "snapshot") {
					cout << "taking snapshot" << endl;
					api.set_buttons(1 << (SNAPSHOT_SWITCH_CHANNEL - 5));
				} else {
					cout << "cmd error: arg was [" << arg << "], format is r {on|off|snapshot}" << endl;
					break;
				}
#endif
		    	    } else if (cmd == 'c') {
				api.click_button(2);
			    } else if (cmd == 'p') {
			      int32_t period;
			      if (!(ss >> period)) {
				cout << "cmd error" << endl;
			      } else {
				cout << "setting reboot period to " << std::dec << period << endl;
				api.set_reboot_period(period);
			      }
			    } else if (cmd == 's') {
		    	    	shutdown = true;
				break;
		    	    }
			}
		}
	}

    // shutdown
	cout << "shutting down..." << endl;
    remove(PIPE_NAME);
	api.disarm();
}
