/*
 * Port.cpp
 *
 *  Created on: Sep 13, 2017
 *      Author: gmoreno
 */

#include "Port.h"
#include <stdio.h>

Port::~Port() {
	// TODO Auto-generated destructor stub
}

void Port::printMessage(mavlink_message_t& message) {
	// Report info
	printf("Received message from port with ID #%d (sys:%d|comp:%d):\n",
			message.msgid, message.sysid, message.compid);
	fprintf(stderr, "Received data: ");
	unsigned int i;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	// check message is write length
	unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
	// message length error
	if (messageLength > MAVLINK_MAX_PACKET_LEN) {
		fprintf(stderr,
				"\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
	} else // print out the buffer
	{
		for (i = 0; i < messageLength; i++) {
			unsigned char v = buffer[i];
			fprintf(stderr, "%02x ", v);
		}
		fprintf(stderr, "\n");
	}
}


// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int
Port::
write_message(const mavlink_message_t &message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	int bytesWritten = _write_port(buf,len);

	return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Convenience Functions
// ------------------------------------------------------------------------------
void
Port::
start()
{
	open_port();
}

void
Port::
stop()
{
	close_port();
}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Port::
handle_quit( int sig )
{
	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop port\n");
	}
}

