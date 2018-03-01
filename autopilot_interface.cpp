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
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <sstream>

using namespace std;

#define RC_IN_MODE_1 1

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	fprintf(stderr, "POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//fprintf(stderr, "VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	fprintf(stderr, "POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(std::shared_ptr<Port> port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // serial port management object

	ticksToReset = 0;
}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					//fprintf(stderr, "MAVLINK_MSG_ID_HEARTBEAT:%x %x\n", current_messages.heartbeat.base_mode, current_messages.heartbeat.custom_mode);
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_EXTENDED_SYS_STATE\n");
					mavlink_msg_extended_sys_state_decode(&message, &(current_messages.extended_sys_state));
					current_messages.time_stamps.extended_sys_state = get_time_usec();
					this_timestamps.extended_sys_state = current_messages.time_stamps.extended_sys_state;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//fprintf(stderr, "MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				case MAVLINK_MSG_ID_HOME_POSITION:
				{
					if (current_messages.time_stamps.home_position == 0) {
						cerr << "Ready!" << endl;
					}
					current_messages.time_stamps.home_position = get_time_usec();
					break;
				}
				default:
				{
					// fprintf(stderr, "Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		fprintf(stderr, "%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		fprintf(stderr, "ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		fprintf(stderr, "\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		fprintf(stderr, "DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		fprintf(stderr, "\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}

/**
 * translate from [-1.0,1.0] to [-1000,1000] truncated if needed
 */
static int16_t translateInput(double x) {
	return std::max(-1.0, std::min(1.0, x)) * 1000;
}

void Autopilot_Interface::send_manual_control(double roll, double pitch, double yaw,
						double thrust, uint16_t buttons) {
	mavlink_message_t msg;
#ifdef RC_IN_MODE_1 
	// this is for mode 1
	mavlink_msg_manual_control_pack(system_id, autopilot_id, &msg, system_id,
			translateInput(roll), translateInput(pitch),
			translateInput(thrust), translateInput(yaw), buttons);
#else
	mavlink_msg_manual_control_pack(system_id, autopilot_id, &msg, system_id,
			translateInput(roll), translateInput(pitch),
			translateInput(yaw), translateInput(thrust), buttons);
#endif
	port->write_message(msg);
}

void Autopilot_Interface::set_manual_control(double roll, double pitch, double yaw, double thrust) {
	current_manual_input.roll = roll;
	current_manual_input.pitch = pitch;
	current_manual_input.thrust = thrust;
	current_manual_input.yaw = yaw;
}

void Autopilot_Interface::click_button(unsigned button) {
	if (button < MAX_BUTTONS) {
#ifdef RC_IN_MODE_1
		current_manual_input.buttons ^=	1 << button;
#else
		button_ticks_left[button] = 1 + WRITE_HZ / 2;
#endif
	}
}

void Autopilot_Interface::setTicksToReset(int ticks) {
	ticksToReset = ticks;
}

void Autopilot_Interface::land() {
	set_flight_mode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND);
}

void Autopilot_Interface::hover() {
	set_manual_control(0, 0, 0, 0.5);
}

void Autopilot_Interface::takeoff() {
	arm();

	mavlink_command_long_t cmd = { 0 };
	cmd.target_system    = system_id;
	cmd.target_component = autopilot_id;
	cmd.command          = MAV_CMD_NAV_TAKEOFF;
	cmd.confirmation     = 0;
	cmd.param1           = -1; // no pitch requested
	cmd.param2 = 0; // not used
	cmd.param3 = 0; // not used
	cmd.param4 = NAN; // yaw angle
	cmd.param5 = NAN; // Y
	cmd.param6 = NAN; // X
	cmd.param7 = 1 + current_messages.global_position_int.alt / 1000; // Z

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &cmd);

	// Send the message
	port->write_message(message);

	hover(); // so that it hovers after taking off
}

int Autopilot_Interface::get_battery_remaining() {
	return current_messages.battery_status.battery_remaining;
}

bool Autopilot_Interface::is_flying() {
	//return current_messages.global_position_int.relative_alt > 100; // flying if higher than 10cm

	// if taking off or landing, consider that flying too
	return current_messages.time_stamps.extended_sys_state > 0
			&& current_messages.extended_sys_state.landed_state != MAV_LANDED_STATE_UNDEFINED
			&& current_messages.extended_sys_state.landed_state != MAV_LANDED_STATE_ON_GROUND;
}

bool Autopilot_Interface::is_home_set() {
	return current_messages.time_stamps.home_position > 0;
}

std::string Autopilot_Interface::get_position() {
	ostringstream pos;
	pos << current_messages.local_position_ned.x;
	pos << ',';
	pos << current_messages.local_position_ned.y;
	pos << ',';
	pos << current_messages.local_position_ned.z;
	pos << ',';
	pos << current_messages.attitude.pitch;
	pos << ',';
	pos << current_messages.attitude.yaw;
	pos << ',';
	pos << current_messages.attitude.roll;
	pos << ',';
	pos << current_messages.local_position_ned.vx;
	pos << ',';
	pos << current_messages.local_position_ned.vy;
	pos << ',';
	pos << current_messages.local_position_ned.vz;
	return pos.str();
}

int
Autopilot_Interface::
arm_disarm( bool arm )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = 0;
	com.param1           = arm ? 1.0 : 0.0;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}

void Autopilot_Interface::arm() {
	arm_disarm(true);
}

void Autopilot_Interface::disarm() {
	arm_disarm(false);
}

uint8_t Autopilot_Interface::get_base_mode() {
	while (current_messages.time_stamps.heartbeat == 0) {
		usleep(100000);
	}
	return current_messages.heartbeat.base_mode;
}

uint8_t Autopilot_Interface::get_main_mode() {
	while (current_messages.time_stamps.heartbeat == 0) {
		usleep(100000);
	}

	px4_custom_mode custom_mode;
	custom_mode.data = current_messages.heartbeat.custom_mode;
	return custom_mode.main_mode;
}

uint8_t Autopilot_Interface::get_sub_mode() {
	while (current_messages.time_stamps.heartbeat == 0) {
		usleep(100000);
	}

	px4_custom_mode custom_mode;
	custom_mode.data = current_messages.heartbeat.custom_mode;
	return custom_mode.sub_mode;
}

bool Autopilot_Interface::mode_equals(uint8_t base_mode, uint8_t main_mode, uint8_t sub_mode) {
	auto _base_mode = get_base_mode();

	return ((_base_mode & base_mode) == base_mode) && get_main_mode() == main_mode
			&& get_sub_mode() == sub_mode;
}

bool Autopilot_Interface::set_flight_mode(uint8_t base_mode, uint8_t main_mode, uint8_t sub_mode) {
	int tries = 3;
	while (tries > 0 && !mode_equals(base_mode, main_mode, sub_mode)) {
		uint8_t _base_mode = get_base_mode();
		uint8_t newBaseMode = _base_mode & ~MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE;
		newBaseMode |= base_mode;

		mavlink_command_long_t cmd = { 0 };
		cmd.target_system = system_id;
		cmd.target_component = autopilot_id;
		cmd.command = MAV_CMD_DO_SET_MODE;
		cmd.confirmation = 1;
		cmd.param1 = newBaseMode;
		cmd.param2 = main_mode;
		cmd.param3 = sub_mode;

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &cmd);

		// Send the message
		port->write_message(message);

		usleep(500000);
		tries--;
	};

//	if (main_mode != get_main_mode()) {
//		cerr << std::hex << "main mode=" << (int) get_main_mode() << " desired=" << (int) main_mode << std::dec << endl;
//	}
//	if (sub_mode != get_sub_mode()) {
//		cerr << std::hex << "sub mode=" << (int) get_sub_mode() << " desired=" << (int) sub_mode << std::dec << endl;
//	}

	return (mode_equals(base_mode, main_mode, sub_mode));
}


void Autopilot_Interface::set_posctl_mode() {
	bool success = set_flight_mode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_POSCTL, 0);
	if (!success) {
		cerr << "set_flight_mode failed" << endl;
	}
}

void Autopilot_Interface::set_reboot_period(int32_t period) {
  mavlink_param_set_t param_set;
  param_set.target_system    = system_id;
  param_set.target_component = autopilot_id;
  strncpy(param_set.param_id, "YL_REBOOT_PERIOD", 16);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
  *((int32_t*) &param_set.param_value) = period;
#pragma GCC diagnostic pop
  param_set.param_type = MAV_PARAM_TYPE_INT32;

  mavlink_message_t message;
  mavlink_msg_param_set_encode(system_id, companion_id, &message, &param_set);

  int len = write_message(message);
  if ( len <= 0 ) {
    fprintf(stderr,"WARNING: could not send PARAM_SET command\n");
  }
  
  return;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( !port->is_open())
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	fprintf(stderr, "START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	fprintf(stderr, "\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	fprintf(stderr, "CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	fprintf(stderr, "Found\n");

	// now we know autopilot is sending messages
	fprintf(stderr, "\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		fprintf(stderr, "GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		fprintf(stderr, "GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		fprintf(stderr, "\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned &&
				  current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	fprintf(stderr, "INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	fprintf(stderr, "INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	fprintf(stderr, "\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	fprintf(stderr, "START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	fprintf(stderr, "\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	fprintf(stderr, "CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	fprintf(stderr, "\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


void Autopilot_Interface::send_heartbeat() {
	mavlink_message_t msg;
		//mavlink_msg_manual_control_pack(system_id, autopilot_id, &msg, system_id,
	mavlink_msg_heartbeat_pack(214, 1, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
			0, // no mode seems relevant
			0,
			0);
	port->write_message(msg);
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	//write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(1000000 / WRITE_HZ);
		//write_setpoint();
		
#ifndef RC_IN_MODE_1
		current_manual_input.buttons = 0;
		for (unsigned b = 0; b < MAX_BUTTONS; b++) {
			if (button_ticks_left[b] > 0) {
				button_ticks_left[b]--;
				current_manual_input.buttons |= 1 << b;
			}
		}
#endif
		if (ticksToReset > 0) {
			ticksToReset--;
			if (ticksToReset == 0) {

				// reset movement so that it hovers
				set_manual_control(0, 0, 0, 0.5);
			}
		}

		send_manual_control(current_manual_input.roll, current_manual_input.pitch,
				current_manual_input.yaw, current_manual_input.thrust,
				current_manual_input.buttons);
		send_heartbeat();
	}

	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



