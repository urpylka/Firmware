/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file sf0x.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Artem Smirnov <urpylka@gmail.com>
 *
 * Driver for the Lightware SF0x laser rangefinder series
 */

#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>

#include <sys/types.h>
// #include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>

// #ifdef __PX4_CYGWIN
// #include <asm/socket.h>
// #endif

#include <perf/perf_counter.h>
// #include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

//#include <board_config.h>

#include "sf0x_parser.h"
#include "tfmini_parser.h"

/* Configuration Constants */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define SF0X_TAKE_RANGE_REG		'd'

// #define SF0X_SAMPLE_PERF		"sf0x_read"
// #define SF0X_COMMS_ERRORS	"sf0x_com_err"
#define SF0X_SAMPLE_PERF		"tfmini_read"
#define SF0X_COMMS_ERRORS		"tfmini_com_err"

// designated SERIAL4/5 on Pixhawk
#define SF0X_DEFAULT_PORT		"/dev/ttyS4"

class SF0X : public cdev::CDev
{
public:
	SF0X(const char *port = SF0X_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~SF0X();

	// настраивает hw_model (границы дистанций, _conversion_interval)
	// создает CDEV устройство
	virtual int				init();

	virtual ssize_t			read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int				ioctl(int filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void					print_info();

private:
	char					_uart_port[20];
	unsigned 				_uart_speed;

	uint8_t					_rotation;
	float					_min_distance;
	float					_max_distance;

	int						_conversion_interval;

	work_s					_work{};
	ringbuffer::RingBuffer	*_reports;
	int						_measure_ticks;
	bool					_collect_phase;

	int						_fd;

	char					_linebuf[10];
	unsigned				_linebuf_index;

	enum TFMINI_PARSE_STATE	tfmini_parse_state;
	enum SF0X_PARSE_STATE 	sf0x_parse_state;
	// enum TFMINI_PARSE_STATE	_parse_state;
	// enum SF0X_PARSE_STATE	_parse_state;
	hrt_abstime				_last_read;

	int						_class_instance;
	int						_orb_class_instance;

	orb_advert_t			_distance_sensor_topic;

	unsigned				_consecutive_fail_count;

	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void					start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void					stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults SF0X_MIN_DISTANCE
	* and SF0X_MAX_DISTANCE
	*/
	void					set_minimum_distance(float min);
	void					set_maximum_distance(float max);
	float					get_minimum_distance();
	float					get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void					cycle();
	int						measure();
	int						collect();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void				cycle_trampoline(void *arg);
};


SF0X::SF0X(const char *port, uint8_t rotation) : CDev(RANGE_FINDER0_DEVICE_PATH)
{
	_rotation = rotation;
	_min_distance = 0.30f;
	_max_distance = 40.0f;
	_conversion_interval = 83334;
	_reports = nullptr;
	_measure_ticks = 0;
	_collect_phase = false;
	_fd = -1;
	_linebuf_index = 0;
	_last_read = 0;
	_class_instance = -1;
	_orb_class_instance = -1;
	_distance_sensor_topic = nullptr;
	_consecutive_fail_count = 0;
	_sample_perf = perf_alloc(PC_ELAPSED, SF0X_SAMPLE_PERF);
	_comms_errors = perf_alloc(PC_COUNT, SF0X_COMMS_ERRORS);

	/* store port name */
	strncpy(_uart_port, port, sizeof(_uart_port) - 1);
	/* enforce null termination */
	_uart_port[sizeof(_uart_port) - 1] = '\0';

	PX4_INFO("_uart_port: %s", _uart_port);
	tfmini_parse_state = TFMINI_PARSE_STATE0_UNSYNC;
	sf0x_parse_state = SF0X_PARSE_STATE0_UNSYNC;
}


SF0X::~SF0X()
{
	/* make sure we are truly inactive */
	SF0X::stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int SF0X::init()
{
	int hw_model;
	param_get(param_find("SENS_EN_SF0X"), &hw_model);
	// hw_model = 7; // DEBUG

	switch (hw_model) {
		case 1: /* SF02 (40m, 12 Hz)*/
			_uart_speed = B115200;

			_min_distance = 0.3f;
			_max_distance = 40.0f;
			_conversion_interval =	83334;
			break;

		case 2:  /* SF10/a (25m 32Hz) */
			_uart_speed = B115200;

			_min_distance = 0.01f;
			_max_distance = 25.0f;
			_conversion_interval = 31250;
			break;

		case 3:  /* SF10/b (50m 32Hz) */
			_uart_speed = B115200;

			_min_distance = 0.01f;
			_max_distance = 50.0f;
			_conversion_interval = 31250;
			break;

		case 4:  /* SF10/c (100m 16Hz) */
			_uart_speed = B115200;

			_min_distance = 0.01f;
			_max_distance = 100.0f;
			_conversion_interval = 62500;
			break;

		case 5:
			/* SF11/c (120m 20Hz) */
			/* if distance sensor model is SF11/C, then set baudrate 115200, else 9600 */
			_uart_speed = B9600;

			_min_distance = 0.01f;
			_max_distance = 120.0f;
			_conversion_interval = 50000;
			break;

		case 6:
			/* SF20/LW20 (100m 48-388Hz) */
			_uart_speed = B115200;

			_min_distance = 0.001f;
			_max_distance = 50.0f;
			_conversion_interval = 20834;
			break;

		case 7: /* TFMINI (12m, 100 Hz)*/
			_uart_speed = B115200;

			_min_distance = 0.3f;
			_max_distance = 12.0f;
			_conversion_interval = 9000;
			break;

		default:
			PX4_ERR("invalid HW model %d.", hw_model);
			return -1;
	}

	PX4_INFO("HW model %d.", hw_model);
	PX4_INFO("_uart_speed: %d", _uart_speed);
	// PX4_INFO("_min_distance: %f", _min_distance);
	// PX4_INFO("_max_distance: %f", _max_distance);
	PX4_INFO("_conversion_interval: %d", _conversion_interval);

	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		// Initialize UART
		if (_fd < 0) { /* fds initialized? */
			/* open fd */
			SF0X::_fd = px4_open(_uart_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
			if (_fd < 0) {
				PX4_ERR("Error opening fd");
				// PX4_ERR("open failed (%i)", errno);
				return -1;
			}

			struct termios uart_config;
			tcgetattr(_fd, &uart_config);				/* fill the struct for the new configuration */

			uart_config.c_oflag &= ~ONLCR;				/* clear ONLCR flag (which appends a CR for every LF) */
			// uart_config.c_cflag &= ~(CSTOPB | PARENB);	/* no parity, one stop bit */

			uart_config.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
			uart_config.c_cflag &= ~CSIZE;
			uart_config.c_cflag |= CS8;         		/* 8-bit characters */
			uart_config.c_cflag &= ~PARENB;     		/* no parity bit */
			uart_config.c_cflag &= ~CSTOPB;     		/* only need 1 stop bit */
			uart_config.c_cflag &= ~CRTSCTS;    		/* no hardware flowcontrol */

			/* setup for non-canonical mode */
			uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
			uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
			uart_config.c_oflag &= ~OPOST;

			/* fetch bytes as they become available */
			uart_config.c_cc[VMIN] = 1;
			uart_config.c_cc[VTIME] = 1;

			int termios_state;
			/* set baud rate */
			if ((termios_state = cfsetispeed(&uart_config, _uart_speed)) < 0) {
				PX4_ERR("CFG: %d ISPD", termios_state);
				ret = -1;
				break;
			}
			if ((termios_state = cfsetospeed(&uart_config, _uart_speed)) < 0) {
				PX4_ERR("CFG: %d OSPD\n", termios_state);
				ret = -1;
				break;
			}
			if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
				PX4_ERR("baud %d ATTR", termios_state);
				ret = -1;
				break;
			}
		}

		/* do regular cdev init */
		ret = CDev::init();
		if (ret != OK) break;

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

		if (_reports == nullptr) {
			PX4_ERR("alloc failed");
			ret = -1;
			break;
		}

		/* set class instance */
		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		/* get a publish handle on the range finder topic */
		struct distance_sensor_s ds_report = {};

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor),
			&ds_report, &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			PX4_ERR("failed to create distance_sensor object. Did you start uOrb?");
		}

	} while (0);

	/* close the fd */
	px4_close(_fd);
	_fd = -1;

	return ret;
}


void SF0X::set_minimum_distance(float min) { _min_distance = min; }
void SF0X::set_maximum_distance(float max) { _max_distance = max; }
float SF0X::get_minimum_distance() { return _min_distance; }
float SF0X::get_maximum_distance() { return _max_distance; }


int SF0X::ioctl(int filp, int cmd, unsigned long arg)
{
	PX4_DEBUG("IO control");
	switch (cmd) {
		case SENSORIOCSPOLLRATE: {
				switch (arg) {
					/* zero would be bad */
					case 0:
						return -EINVAL;

					/* set default polling rate */
					case SENSOR_POLLRATE_DEFAULT: {
						bool want_start = (_measure_ticks == 0);			/* do we need to start internal polling? */
						_measure_ticks = USEC2TICK(_conversion_interval);	/* set interval for next measurement to minimum legal value */
						if (want_start)	SF0X::start();						/* if we need to start the poll state machine, do it */
						return OK;
					}

					/* adjust to a legal polling interval in Hz */
					default: {
						bool want_start = (_measure_ticks == 0);			/* do we need to start internal polling? */

						int ticks = USEC2TICK(1000000 / arg);				/* convert hz to tick interval via microseconds */
						if (ticks < USEC2TICK(_conversion_interval)) return -EINVAL;	/* check against maximum rate */

						_measure_ticks = ticks;								/* update interval for next measurement */
						if (want_start) SF0X::start();						/* if we need to start the poll state machine, do it */
						return OK;
					}
				}
		}

		default:
			/* give it to the superclass */
			return px4_ioctl(filp, cmd, arg);
	}
}


ssize_t SF0X::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		// deprecate urpylka
		// /* trigger a measurement */
		// if (OK != measure()) {
		// 	ret = -EIO;
		// 	break;
		// }

		/* wait for it to complete */
		px4_usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != SF0X::collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) ret = sizeof(*rbuf);

	} while (0);

	return ret;
}


int SF0X::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	char cmd = SF0X_TAKE_RANGE_REG;
	ret = px4_write(_fd, &cmd, 1);

	if (ret != sizeof(cmd)) {
		perf_count(_comms_errors);
		PX4_DEBUG("write fail %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}


int SF0X::collect()
{
	int ret = 0;
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	float distance_m = -1.0f;
	bool valid = false;

	/* Check the number of bytes available in the buffer*/
	int bytes_available = 0;
	SF0X::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
	if (!bytes_available) return -EAGAIN;

	/* parse entire buffer */
	do {
		/* read from the sensor (uart buffer) */
		ret = px4_read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_DEBUG("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			/* only throw an error if we time out */
			if (read_elapsed > (_conversion_interval * 2)) {
					/* flush anything in RX buffer */
					tcflush(_fd, TCIFLUSH);
					return ret;
				} else return -EAGAIN;
		} else if (ret == 0) return -EAGAIN;

		_last_read = hrt_absolute_time();

		for (int i = 0; i < ret; i++) {
			if (OK == tfmini_parse(readbuf[i], _linebuf, &_linebuf_index, &tfmini_parse_state, &distance_m)) {
			// if (OK == sf0x_parser(readbuf[i], _linebuf, &_linebuf_index, &sf0x_parse_state, &distance_m)) {
				valid = true;
			}
		}

		/* bytes left to parse */
		bytes_available -= ret;

	} while (bytes_available > 0);

	/* no valid measurement after parsing buffer */
	if (distance_m < 0.0f) return -EAGAIN;

	PX4_DEBUG("val (float): %8.4f, raw: %s, valid: %s", (double)distance_m, _linebuf, ((valid) ? "OK" : "NO"));

	/* publish most recent valid measurement from buffer */
	struct distance_sensor_s report{};

	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = _rotation;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.variance = 0.0f;
	report.signal_quality = -1;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it */
	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}


void SF0X::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&SF0X::cycle_trampoline, this, 1);
}


void SF0X::stop()
{
	work_cancel(HPWORK, &_work);
}


void SF0X::cycle_trampoline(void *arg)
{
	SF0X *dev = static_cast<SF0X *>(arg);
	dev->cycle();
}


void SF0X::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = SF0X::collect();

		if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
			work_queue(HPWORK, &_work, (worker_t)&SF0X::cycle_trampoline, this, USEC2TICK(1042 * 8));
			return;
		}


		if (OK != collect_ret) {

			/* we know the sensor needs about four seconds to initialize */
			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
				PX4_ERR("collection error #%u", _consecutive_fail_count);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			SF0X::start();
			return;

		} else {
			/* apparently success */
			_consecutive_fail_count = 0;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(_conversion_interval)) {
			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK, &_work, (worker_t)&SF0X::cycle_trampoline, this, _measure_ticks - USEC2TICK(_conversion_interval));
			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) PX4_DEBUG("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK, &_work, (worker_t)&SF0X::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}


void SF0X::print_info()
{
	printf("Using port '%s'\n", _uart_port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %d ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}
