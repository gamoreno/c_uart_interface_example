/*
 * Port.h
 *
 *  Created on: Sep 13, 2017
 *      Author: gmoreno
 */

#ifndef PORT_H_
#define PORT_H_

#include <common/mavlink.h>
#include <pthread.h>


// Status flags
#define PORT_OPEN   1;
#define PORT_CLOSED 0;
#define PORT_ERROR -1;



class Port {
public:
	virtual ~Port();

	virtual int read_message(mavlink_message_t &message);
	virtual int write_message(const mavlink_message_t &message);

	virtual void start();
	virtual void stop();

	virtual void handle_quit( int sig );

	inline virtual bool is_open() const { return status == PORT_OPEN; }

protected:
	virtual void open_port() = 0;
	virtual void close_port() = 0;

	virtual int _read_port(uint8_t &cp) = 0;
	virtual int _write_port(char *buf, unsigned len) = 0;

	bool debug;
	int  status;
	mavlink_status_t lastStatus;
	pthread_mutex_t  lock;

};

#endif /* PORT_H_ */
