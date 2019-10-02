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
 * @file tfmini_nsh_commands.cpp
 * @author Artem Smirnov <urpylka@gmail.com>
 *
 * Driver for the TFmini laser rangefinder series
 */

#include <px4_module.h>

#include "tfmini.cpp"

/**
 * Local functions in support of the shell command.
 */
namespace tfmini
{
	TFMINI *g_dev;

	int start(const char *port, uint8_t rotation);
	int stop();
	int test();
	int info();
	void usage();

	/**
	 * Start the driver.
	 */
	int start(const char *port, uint8_t rotation)
	{
		// Initialize TFMINI
		if (g_dev != nullptr) {
			PX4_WARN("already started");
			return -1;
		}

		// needs for use "goto fail;"
		int fd;

		/* create the driver */
		g_dev = new TFMINI(port, rotation);

		if (OK != g_dev->init()) {
			PX4_ERR("OK != g_dev->init()");
			goto fail;
		}

		if (g_dev == nullptr) {
			PX4_ERR("g_dev == nullptr");
			goto fail;
		}

		// RANGE_FINDER0_DEVICE_PATH declarate in drv_range_finder.h
		//before used open() from fs/vfs/fs_open.c
		fd = px4_open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("device open fail (%i)", errno);
			goto fail;
		}

		/* set the poll rate to default, starts automatic data collection */
		// before used ioctl() from libs/libc/misc/lib_ioctl.c
		// maybe correct to use TFMINI::ioctl
		if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			PX4_ERR("ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0");
			goto fail;
		}
		return 0;

		fail:
		if (g_dev != nullptr) { delete g_dev; g_dev = nullptr; }
		PX4_ERR("driver start failed");
		return 1;
	}


	/**
	 * Stop the driver
	 */
	int stop()
	{
		if (g_dev != nullptr) {
			PX4_INFO("stopping driver");
			delete g_dev;
			g_dev = nullptr;
			PX4_INFO("driver stopped");
		} else {
			PX4_ERR("driver not running");
			return 1;
		}

		return 0;
	}


	/**
	 * Perform some basic functional tests on the driver;
	 * make sure we can collect data from the sensor in polled
	 * and automatic modes.
	 */
	int test()
	{
		unsigned int count_of_messages = 1;
		struct distance_sensor_s report;
		ssize_t sz;
		int fd = px4_open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("%s open failed (try 'tfmini start' if the driver is not running", RANGE_FINDER0_DEVICE_PATH);
			return 1;
		}

		/* do a simple demand read */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("immediate read failed. sz: %d, sizeof(report): %d", sz, sizeof(report));
			close(fd);
			return 1;
		}

		print_message(report);

		/* start the sensor polling at 2 Hz rate */
		if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
			PX4_ERR("failed to set 2Hz poll rate");
			return 1;
		}


		/* read the sensor 5x and report each value */
		for (unsigned i = 1; i < count_of_messages; i++) {
			struct pollfd fds;

			/* wait for data to be ready */
			fds.fd = fd;
			fds.events = POLLIN;
			int ret = px4_poll(&fds, 1, 2000);

			if (ret != 1) {
				PX4_ERR("px4_poll: timed out");
				break;
			}

			/* now go get it */
			sz = px4_read(fd, &report, sizeof(report));

			if (sz != sizeof(report)) {
				PX4_ERR("read failed: got %zi vs exp. %zu", sz, sizeof(report));
				break;
			}

			print_message(report);
		}

		/* reset the sensor polling to the default rate */
		if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
			PX4_ERR("ioctl SENSORIOCSPOLLRATE failed");
			return 1;
		}

		PX4_INFO("PASS");
		return 0;
	}


	/**
	 * Print a little info about the driver.
	 */
	int info()
	{
		if (g_dev == nullptr) {
			PX4_ERR("driver not running");
			return -1;
		}

		printf("state @ %p\n", g_dev);
		g_dev->print_info();

		return 0;
	}


	/**
	 * Print a little info on how to use the driver.
	 */
	void usage()
	{
		PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
	### Description

	Serial bus driver for the Benewake TFmini LiDAR.
	Most boards are configured to enable/start the driver on a specified UART using the SENS_TFMINI_CFG parameter.
	Setup/usage information: https://docs.px4.io/en/sensor/tfmini.html

	### Examples

	Attempt to start driver on a specified serial device.
	$ tfmini start -d /dev/ttyS1
	Stop driver
	$ tfmini stop
	)DESCR_STR");

		PRINT_MODULE_USAGE_NAME("tfmini", "driver");
		PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
		PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
		PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
		PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
		PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
		PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");
		PRINT_MODULE_USAGE_COMMAND_DESCR("info","Print driver information");
		PRINT_MODULE_USAGE_COMMAND_DESCR("usage","Print usage information");
	}
} // namespace


/*
 * Driver 'main' command.
 */

extern "C" __EXPORT int tfmini_main(int argc, char *argv[]);

int tfmini_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = "";
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return -1;
		}
	}

	if (myoptind >= argc) goto out_error;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return tfmini::start(device_path, rotation);

		} else {
			PX4_WARN("Please specify device path!");
			tfmini::usage();
			return 1;
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return tfmini::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return tfmini::test();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info")) {
		return tfmini::info();
	}

	/*
	 * Print usage information.
	 */
	if (!strcmp(argv[myoptind], "usage")) {
		tfmini::usage();
	}

	out_error:
	PX4_ERR("unrecognized command");
	tfmini::usage();
	return 1;
}
