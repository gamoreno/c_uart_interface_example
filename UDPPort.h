/*
 * UDPPort.h
 *
 *  Created on: Sep 13, 2017
 *      Author: gmoreno
 */

#ifndef UDPPORT_H_
#define UDPPORT_H_

#include "Port.h"
#include <string>

class UDP_Port : public Port {
public:
	UDP_Port(const std::string& host, const std::string& port, const std::string& localPort);
	virtual ~UDP_Port();

	virtual int read_message(mavlink_message_t &message);

protected:
	virtual void open_port();
	virtual void close_port();
	virtual int _write_port(char *buf, unsigned len);

	std::string host;
	std::string port;
	std::string localPort;

	int sockfd = -1;
	struct addrinfo* remoteAddress = 0;

private:
	int _read_port(uint8_t* buffer, size_t bufferLen);
};

#endif /* UDPPORT_H_ */
