/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <fstream>
#include <sstream>
#include <common/mavlink.h>
#include <memory>

#include "utils.h"
#include "serial_port.h"
#include "UDPPort.h"
#include "autopilot_interface.h"
#include "pipe_control.h"
#include "stdio_control.h"

using namespace std;


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top();

void commands(Autopilot_Interface &autopilot_interface);
bool parse_commandline(int argc, char **argv);

// quit handler
Autopilot_Interface *autopilot_interface_quit;
void quit_handler(int sig);

std::shared_ptr<Port> port;
bool useStdio = false;

#define REBOOT_SWITCH_CHANNEL 6
#define SNAPSHOT_SWITCH_CHANNEL 7


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top()
{

	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Autopilot_Interface autopilot_interface(port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	port->start();
	autopilot_interface.start();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	//commands(autopilot_interface);
	if (useStdio) {
		stdio_control(autopilot_interface);
	} else {
		remoteCommands(autopilot_interface);
	}


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	port->stop();


	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------


void test_manual_control(Autopilot_Interface &api, double roll, double pitch, double yaw, double thrust) {
	api.set_manual_control(roll, pitch, yaw, thrust);
	sleep(1);
	api.set_manual_control(0.0, 0.0, 0.0, thrust);
	sleep(3);
}


void
commands(Autopilot_Interface &api)
{
	const double v = 0.5;
	const double HOVER_THRUST = 0.5;
	unsigned delay = 1;

	api.arm();
	sleep(2);

	api.set_posctl_mode();
	sleep(2);


	api.set_manual_control(0.0, 0.0, 0.0, 1.0);
	sleep(delay);
	api.set_manual_control(0.0, 0.0, 0.0, HOVER_THRUST);
	sleep(3);

	test_manual_control(api, -v, 0.0, 0.0, HOVER_THRUST);
	test_manual_control(api, v, 0.0, 0.0, HOVER_THRUST);
	test_manual_control(api, 0.0, -v, 0.0, HOVER_THRUST);
	test_manual_control(api, 0.0, v, 0.0, HOVER_THRUST);
	test_manual_control(api, 0.0, 0.0, -v, HOVER_THRUST);
	test_manual_control(api, 0.0, 0.0, v, HOVER_THRUST);

	api.set_manual_control(0.0, 0.0, 0.0, 0.0);
	sleep(6);

	api.disarm();

	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
	printf("    ap time:     %lu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	printf("    temperature: %f C \n"       , imu.temperature );

	printf("\n");


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
bool
parse_commandline(int argc, char **argv)
{
	// string for command line usage
	const char *commandline_usage = "usage: mavlink_control [-s] serial[:device[:baud]]|udp[:host[:port[:localPort]]]";
	bool cmdLineError = false;

	for (int arg = 1; arg < argc; arg++) {
		if (argv[arg][0] == '-') {
			if (argv[arg][1] == 's') {
				useStdio = true;
			} else {
				cmdLineError = true;
			}
		} else {
			auto args = splitString(argv[arg], ":");
			if (args[0] == "serial") {

				// defaults
	#ifdef __APPLE__
				const char *uart_name = (char*)"/dev/tty.usbmodem1";
	#else
				const char *uart_name = (char*)"/dev/ttyUSB0";
	#endif
				int baudrate = 57600;

				if (args.size() > 1) {
					uart_name = args[1].c_str();
				}
				if (args.size() > 2) {
					baudrate = atoi(args[2].c_str());
				}
				port = std::make_shared<Serial_Port>(uart_name, baudrate);
			} else if (args[0] == "udp") {

				// defaults
				string host = "localhost";
				string remotePort = "14557";
				string localPort = "14540";

				if (args.size() > 1) {
					host = args[1];
				}
				if (args.size() > 2) {
					remotePort = args[2];
				}
				if (args.size() > 3) {
					localPort = args[3];
				}

				port = std::make_shared<UDP_Port>(host, remotePort, localPort);
			}
		}
	}

	if (cmdLineError ||  !port) {
		printf("%s\n", commandline_usage);
		return false;
	}

	return true;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		port->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// parse command line and instantiate a port
	if (!parse_commandline(argc,argv)) {
		return 1;
	}

	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top();
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return 1;
	}
}


