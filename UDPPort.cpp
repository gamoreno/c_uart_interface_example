/*
 * UDPPort.cpp
 *
 *  Created on: Sep 13, 2017
 *      Author: gmoreno
 */

#include "UDPPort.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

UDP_Port::UDP_Port(const std::string& host, const std::string& port, const std::string& localPort)
	: host(host), port(port), localPort(localPort)
{
	status = PORT_CLOSED;
	//debug = true;
}

UDP_Port::~UDP_Port() {
	// TODO Auto-generated destructor stub
}

void UDP_Port::open_port() {
	if (is_open()) {
		printf("error port is already open!\n");
		throw EXIT_FAILURE;
	}

	struct addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_protocol = 0;
	hints.ai_flags = AI_ADDRCONFIG;

	int err = getaddrinfo(host.c_str(), port.c_str(), &hints, &remoteAddress);
	if (err != 0) {
	    printf("Error resolving remote address: %s\n", gai_strerror(err));
        throw EXIT_FAILURE;
	}

    sockfd = socket(remoteAddress->ai_family, remoteAddress->ai_socktype, remoteAddress->ai_protocol);
    if (sockfd < 0) {
        printf("Error opening socket: %s\n", strerror(errno));
        throw EXIT_FAILURE;
    }

    // bind to local port to receive
    struct addrinfo* localAddress = 0;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_protocol = 0;
	hints.ai_flags = AI_PASSIVE | AI_ADDRCONFIG;

	err = getaddrinfo(NULL, localPort.c_str(), &hints, &localAddress);
	if (err != 0) {
	    printf("Error resolving remote address: %s\n", gai_strerror(err));
        throw EXIT_FAILURE;
	}

    if (bind(sockfd, localAddress->ai_addr, localAddress->ai_addrlen)==-1) {
        printf("Error binding socket: %s\n", strerror(errno));
        throw EXIT_FAILURE;
    }

    freeaddrinfo(localAddress);

    status = PORT_OPEN;
}

void UDP_Port::close_port() {
	if (!is_open()) {
		return;
	}
	close(sockfd);
	freeaddrinfo(remoteAddress);

	status = PORT_CLOSED;
}

int UDP_Port::_read_port(uint8_t* buffer, size_t bufferLen) {
	ssize_t result = recvfrom(sockfd, buffer, bufferLen, 0, NULL, NULL);
	return result;
}

int
UDP_Port::
read_message(mavlink_message_t &message)
{
	const size_t BUFFER_LEN = 300;
	uint8_t buffer[BUFFER_LEN];
	mavlink_status_t status;
	uint8_t msgReceived = false;

	// --------------------------------------------------------------------------
	//   READ FROM PORT
	// --------------------------------------------------------------------------

	int result = _read_port(buffer, BUFFER_LEN);

	if (result > 0)
	{
		for (int idx = 0; idx < result; idx++) {
			//printf("parseState=%d data=%02x\n", status.parse_state, buffer[idx]);
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, buffer[idx], &message, &status);

			// check for dropped packets
			if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
			{
				printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				fprintf(stderr,"%02x ", buffer[idx]);
			}
			lastStatus = status;

			if(msgReceived && debug)
			{
				// Report info
				printMessage(message);
				return msgReceived;
			}
		}
	}

	// Couldn't read from port
	else
	{
		fprintf(stderr, "ERROR: Could not read from port\n");
	}

	return msgReceived;
}


int UDP_Port::_write_port(char* buf, unsigned len) {

	const int bytesWritten = sendto(sockfd, buf, len, 0, remoteAddress->ai_addr, remoteAddress->ai_addrlen);

	return bytesWritten;
}
