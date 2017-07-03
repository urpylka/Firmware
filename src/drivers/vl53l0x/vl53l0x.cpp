/****************************************************************************
 *
 *   Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
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
 * @file vl53l0x.cpp
 * @author Tarasov Elia <elias.tarasov@gmail.com>
 *
 * Driver for the STM vl53l0x laser rangefinder series
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
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
#include <vector>
#include <array>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#define VL53L0X_BUS 		PX4_I2C_BUS_EXPANSION
#define VL53L0X_BASEADDR 	0x29
#define VL53L0X_DEVICE_PATH	"/dev/vl53l0x"

/* Register's map */
#define VL53L0X_WHO_AM_I 0xC0
#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N 0xBF
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV 0x89
#define MSRC_CONFIG_CONTROL 0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define SYSRANGE_START	0x00
#define GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define SYSTEM_SEQUENCE_CONFIG 0x01
#define SYSTEM_RANGE_CONFIG 0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD 0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define SYSTEM_INTERRUPT_CLEAR 0x0B
#define RESULT_INTERRUPT_STATUS 0x13
#define RESULT_RANGE_STATUS 0x14
#define OSC_CALIBRATE_VAL 0xF8
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1 0xB1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2 0xB2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3 0xB3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4 0xB4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5 0xB5
#define DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define MSRC_CONFIG_TIMEOUT_MACROP 0x46
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x52
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x72
#define PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x52
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x72
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH 0x57
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW 0x56
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW 0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x48
#define GLOBAL_CONFIG_VCSEL_WIDTH 0x32
#define ALGO_PHASECAL_LIM 0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT 0x30

#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class VL53L0X : public device::I2C
{
public:
	VL53L0X(int bus = VL53L0X_BUS, int address = VL53L0X_BASEADDR);
	virtual ~VL53L0X();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	float				_min_distance;
	float				_max_distance;
	int                 _conversion_interval;
	work_s				_work;
	ringbuffer::RingBuffer  *_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	int					_class_instance;
	int					_orb_class_instance;

	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint16_t 			timeout_start_ms;
	uint16_t 			io_timeout_ms;
	uint8_t 			stop_variable;
	uint32_t 			measurement_timing_budget_us;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults SF1XX_MIN_DISTANCE
	* and SF1XX_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	int					writeReg(uint8_t reg, uint8_t data);
	int					writeReg16(uint8_t reg, uint16_t data);
	int					writeReg32(uint8_t reg, uint32_t data);
	uint8_t				readReg(uint8_t reg);
	uint16_t			readReg16(uint8_t reg);
	uint32_t			readReg32(uint8_t reg);
	void				writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
	void 				readMulti(uint8_t reg, uint8_t * dst, uint8_t count);
	bool 				setSignalRateLimit(float limit_Mcps);
	float 				getSignalRateLimit(void);

	// Record the current time in milliseconds to check an upcoming timeout against
	void startTimeout() {
		timeout_start_ms = hrt_absolute_time()/1000;
	}

	// Check if timeout is enabled (set to nonzero value) and has expired
	bool checkTimeoutExpired() {
		if( (io_timeout_ms > 0) && (static_cast<uint16_t>(hrt_absolute_time()/1000) - timeout_start_ms) > io_timeout_ms)
			return true;
		return false;
	}

	bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

	bool setMeasurementTimingBudget(uint32_t budget_us);
	uint32_t getMeasurementTimingBudget(void);

	struct SequenceStepEnables
	{
		bool tcc, msrc, dss, pre_range, final_range;
	};

	struct SequenceStepTimeouts
	{
	      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
	      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
	      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
	};

	enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

	void getSequenceStepEnables(SequenceStepEnables * enables);
	void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

	uint8_t getVcselPulsePeriod(vcselPeriodType type);
	bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

	static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
	static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

    static uint16_t decodeTimeout(uint16_t value);
    static uint16_t encodeTimeout(uint16_t timeout_mclks);

    bool performSingleRefCalibration(uint8_t vhv_init_byte);

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int vl53l0x_main(int argc, char *argv[]);

VL53L0X::VL53L0X(int bus, int address) :
	I2C("VL53L0X", VL53L0X_DEVICE_PATH, bus, address, 400000),
	_min_distance(-1.0f),
	_max_distance(-1.0f),
	_conversion_interval(-1),
	_work{},
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "vl53l0x_read")),
	_comms_errors(perf_alloc(PC_COUNT, "vl53l0x_com_err")),
	timeout_start_ms(hrt_absolute_time()*1000),
	io_timeout_ms(500),
	stop_variable(0),
	measurement_timing_budget_us(0)
{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

VL53L0X::~VL53L0X()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
VL53L0X::init()
{
	int ret = PX4_ERROR;

	int hw_model;
	param_get(param_find("SENS_EN_VL53L0X"), &hw_model);

	switch (hw_model) {
	case 0:
		DEVICE_LOG("disabled.");
		return ret;

	case 1:
		_min_distance = 0.01f;
		_max_distance = 2.0f;
		_conversion_interval = 10000;
		break;

	default:
		DEVICE_LOG("invalid HW model %d.", hw_model);
		return ret;
	}

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	/* set device base address */
	set_address(VL53L0X_BASEADDR);

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report, &_orb_class_instance, ORB_PRIO_HIGH);

	if (_distance_sensor_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}

	// read WHO_AM_I register
	uint8_t I_AM = readReg(VL53L0X_WHO_AM_I);
	if(I_AM == 0xEE) {
		// "Set I2C standard mode"
		writeReg(0x88, 0x00);
		writeReg(0x80, 0x01);
		writeReg(0xFF, 0x01);
		writeReg(0x00, 0x00);
		stop_variable = readReg(0x91);
		writeReg(0x01, 0x00);
		writeReg(0xFF, 0x00);
		writeReg(0x80, 0x00);

		// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
		writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

		setSignalRateLimit(0.1);
		setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
		setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

		writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

		uint8_t spad_count;
		bool spad_type_is_aperture;
		if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) {
			printf("No spad info\n");
			return false;
		}

		// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
		// the API, but the same data seems to be more easily readable from
		// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
		uint8_t ref_spad_map[6];
		readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

		// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

		writeReg(0xFF, 0x01);
		writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
		writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
		writeReg(0xFF, 0x00);
		writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

		uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
		uint8_t spads_enabled = 0;

		for (uint8_t i = 0; i < 48; i++) {
			if (i < first_spad_to_enable || spads_enabled == spad_count) {
		      // This bit is lower than the first one that should be enabled, or
		      // (reference_spad_count) bits have already been enabled, so zero this bit
		      ref_spad_map[i / 8] &= ~(1 << (i % 8));
		    }
		    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
		    {
		      spads_enabled++;
		    }
		}

		writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

		writeReg(0xFF, 0x01);
		writeReg(0x00, 0x00);

		writeReg(0xFF, 0x00);
		writeReg(0x09, 0x00);
		writeReg(0x10, 0x00);
		writeReg(0x11, 0x00);

		writeReg(0x24, 0x01);
		writeReg(0x25, 0xFF);
		writeReg(0x75, 0x00);

		writeReg(0xFF, 0x01);
		writeReg(0x4E, 0x2C);
		writeReg(0x48, 0x00);
		writeReg(0x30, 0x20);

		writeReg(0xFF, 0x00);
		writeReg(0x30, 0x09);
		writeReg(0x54, 0x00);
		writeReg(0x31, 0x04);
		writeReg(0x32, 0x03);
		writeReg(0x40, 0x83);
		writeReg(0x46, 0x25);
		writeReg(0x60, 0x00);
		writeReg(0x27, 0x00);
		writeReg(0x50, 0x06);
		writeReg(0x51, 0x00);
		writeReg(0x52, 0x96);
		writeReg(0x56, 0x08);
		writeReg(0x57, 0x30);
		writeReg(0x61, 0x00);
		writeReg(0x62, 0x00);
		writeReg(0x64, 0x00);
		writeReg(0x65, 0x00);
		writeReg(0x66, 0xA0);

		writeReg(0xFF, 0x01);
		writeReg(0x22, 0x32);
		writeReg(0x47, 0x14);
		writeReg(0x49, 0xFF);
		writeReg(0x4A, 0x00);

		writeReg(0xFF, 0x00);
		writeReg(0x7A, 0x0A);
		writeReg(0x7B, 0x00);
		writeReg(0x78, 0x21);

		writeReg(0xFF, 0x01);
		writeReg(0x23, 0x34);
		writeReg(0x42, 0x00);
		writeReg(0x44, 0xFF);
		writeReg(0x45, 0x26);
		writeReg(0x46, 0x05);
		writeReg(0x40, 0x40);
		writeReg(0x0E, 0x06);
		writeReg(0x20, 0x1A);
		writeReg(0x43, 0x40);

		writeReg(0xFF, 0x00);
		writeReg(0x34, 0x03);
		writeReg(0x35, 0x44);

		writeReg(0xFF, 0x01);
		writeReg(0x31, 0x04);
		writeReg(0x4B, 0x09);
		writeReg(0x4C, 0x05);
		writeReg(0x4D, 0x04);

		writeReg(0xFF, 0x00);
		writeReg(0x44, 0x00);
		writeReg(0x45, 0x20);
		writeReg(0x47, 0x08);
		writeReg(0x48, 0x28);
		writeReg(0x67, 0x00);
		writeReg(0x70, 0x04);
		writeReg(0x71, 0x01);
		writeReg(0x72, 0xFE);
		writeReg(0x76, 0x00);
		writeReg(0x77, 0x00);

		writeReg(0xFF, 0x01);
		writeReg(0x0D, 0x01);

		writeReg(0xFF, 0x00);
		writeReg(0x80, 0x01);
		writeReg(0x01, 0xF8);

		writeReg(0xFF, 0x01);
		writeReg(0x8E, 0x01);
		writeReg(0x00, 0x01);
		writeReg(0xFF, 0x00);
		writeReg(0x80, 0x00);

		writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
		writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
		writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

		measurement_timing_budget_us = getMeasurementTimingBudget();

		// "Disable MSRC and TCC by default"
		// MSRC = Minimum Signal Rate Check
		// TCC = Target CentreCheck
		// -- VL53L0X_SetSequenceStepEnable() begin

		writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

		// -- VL53L0X_SetSequenceStepEnable() end

		// "Recalculate timing budget"
		setMeasurementTimingBudget(20000);

		// VL53L0X_StaticInit() end

		// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

		// -- VL53L0X_perform_vhv_calibration() begin

		writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
		if (!performSingleRefCalibration(0x40)) {
			DEVICE_LOG("(%dHz) with address 0x%02x performSingleRefCalibration failed", (int)(1e6f / _conversion_interval), VL53L0X_BASEADDR);
			return false;
		}

		// -- VL53L0X_perform_vhv_calibration() end

		// -- VL53L0X_perform_phase_calibration() begin

		writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (!performSingleRefCalibration(0x00)) {
			DEVICE_LOG("(%dHz) with address 0x%02x performSingleRefCalibration failed ", (int)(1e6f / _conversion_interval), VL53L0X_BASEADDR);
			return false;
		}

		// -- VL53L0X_perform_phase_calibration() end

		// "restore the previous Sequence Config"
		writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

		DEVICE_LOG("(%dm %dHz) with address 0x%02x found", (int)_max_distance, (int)(1e6f / _conversion_interval), VL53L0X_BASEADDR);
		// VL53L0X_PerformRefCalibration() end

		ret = PX4_OK;
	}
	return ret;
}

int VL53L0X::writeReg(uint8_t reg, uint8_t value)
{
	std::array<uint8_t, 2> val;
	val[0] = reg;
	val[1] = value;
	int isOK = transfer(val.data(), 2, nullptr, 0);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: write register 0x%02x failed", VL53L0X_BASEADDR, reg);
		return -1;
	}
	return OK;
}

int VL53L0X::writeReg16(uint8_t reg, uint16_t value)
{
	std::array<uint8_t, 3> val;
	val[0] = reg;
	val[1] = (value >> 8) & 0xFF; // value high byte
	val[2] =  value       & 0xFF; // value low byte
	int isOK = transfer(val.data(), 3, nullptr, 0);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: write register 0x%02x failed", VL53L0X_BASEADDR, reg);
		return -1;
	}
	return OK;
}

int VL53L0X::writeReg32(uint8_t reg, uint32_t value)
{
	std::array<uint8_t, 5> val;
	val[0] = reg;
	val[1] = (value >> 24) & 0xFF; // value high byte
	val[2] = (value >> 16) & 0xFF;
	val[3] = (value >>  8) & 0xFF;
	val[4] =  value        & 0xFF; // value low byte
	int isOK = transfer(val.data(), 5, nullptr, 0);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: write register 0x%02x failed", VL53L0X_BASEADDR, reg);
		return -1;
	}
	return OK;
}

uint8_t VL53L0X::readReg(uint8_t reg)
{
	std::array<uint8_t, 1> write_reg;
	std::array<uint8_t, 1> read_val;
	write_reg[0] = reg;
	int isOK = transfer(write_reg.data(), 1, &read_val[0], 1);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: read register 0x%02x failed", VL53L0X_BASEADDR, reg);
		return 0;
	}
	return read_val[0];
}

uint16_t VL53L0X::readReg16(uint8_t reg)
{
	uint8_t write_reg[1] = {reg};
	uint8_t read_val[2] = {0,0};
	int isOK = transfer(write_reg, 1, &read_val[0], 2);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: read register 0x%02x failed", VL53L0X_BASEADDR, reg);
		return 0;
	}
	uint16_t value;
	value  = (uint16_t)(read_val[0] << 8); // value high byte
	value |= read_val[1]; 				 // value low byte
	return value;
}

// Read a 32-bit register
uint32_t VL53L0X::readReg32(uint8_t reg)
{
	std::array<uint8_t, 1> write_reg;
	std::array<uint8_t, 4> read_val;
	write_reg[0] = reg;
	int isOK = transfer(write_reg.data(), 1, &read_val[0], 4);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: read register 0x%02x failed", VL53L0X_BASEADDR, reg);
		return 0;
	}
	uint32_t value;
	value  = (uint32_t)read_val[0] << 24; // value highest byte
	value |= (uint32_t)read_val[1] << 16;
	value |= (uint16_t)read_val[2] <<  8;
	value |=           read_val[3];       // value lowest byte

	return value;
}

void VL53L0X::writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
	std::vector<uint8_t> data(count+1);
	data[0] = reg;
	memcpy(&data[1], src, count);
	int isOK = transfer(data.data(), count+1, nullptr, 0);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: write register 0x%02x failed", VL53L0X_BASEADDR, reg);
	}
}

void VL53L0X::readMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{
	std::array<uint8_t, 1> read_reg;
	read_reg[0] = reg;
	int isOK = transfer(read_reg.data(), 1, dst, count);
	if(isOK != OK) {
		DEVICE_LOG("Device 0x%02x: read register 0x%02x failed", VL53L0X_BASEADDR, reg);
	}
}

bool VL53L0X::setSignalRateLimit(float limit_Mcps)
{
	if (limit_Mcps < 0.0f || limit_Mcps > 511.99f) { return false; }

	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	writeReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
	return true;
}

// Get the return signal rate limit check value in MCPS
float VL53L0X::getSignalRateLimit(void)
{
  return (float)readReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X::getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6b);
  writeReg(0x83, 0x00);
  startTimeout();
  while (readReg(0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
  writeReg(0x83, 0x01);
  tmp = readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg( 0x83  & ~0x04));
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return true;
}

uint32_t VL53L0X::getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool VL53L0X::setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
      final_range_timeout_mclks += timeouts.pre_range_mclks;

    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

void VL53L0X::getSequenceStepEnables(SequenceStepEnables * enables) {
	uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
	enables->tcc          = (sequence_config >> 4) & 0x1;
	enables->dss          = (sequence_config >> 3) & 0x1;
	enables->msrc         = (sequence_config >> 2) & 0x1;
	enables->pre_range    = (sequence_config >> 6) & 0x1;
	enables->final_range  = (sequence_config >> 7) & 0x1;
}

void VL53L0X::getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts) {
	timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

	timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

	timeouts->pre_range_mclks = decodeTimeout(readReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

	timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

	timeouts->final_range_mclks = decodeTimeout(readReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	if (enables->pre_range)
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;

	timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

uint8_t VL53L0X::getVcselPulsePeriod(vcselPeriodType type) {
	if (type == VcselPeriodPreRange)
		return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
	else if (type == VcselPeriodFinalRange)
		return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	else return 255;
}


bool VL53L0X::setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) {

	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	getSequenceStepEnables(&enables);
	getSequenceStepTimeouts(&enables, &timeouts);

	// "Apply specific settings for the requested clock period"
	// "Re-calculate and apply timeouts, in macro periods"

	// "When the VCSEL period for the pre or final range is changed,
	// the corresponding timeout must be read from the device using
	// the current VCSEL period, then the new VCSEL period can be
	// applied. The timeout then must be written back to the device
	// using the new VCSEL period.
	//
	// For the MSRC timeout, the same applies - this timeout being
	// dependant on the pre-range vcsel period."

	if (type == VcselPeriodPreRange) {
	    // "Set phase check limits"
	    switch (period_pclks) {
	      case 12:
	        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
	        break;

	      case 14:
	        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
	        break;

	      case 16:
	        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
	        break;

	      case 18:
	        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
	        break;

	      default:
	        // invalid period
	        return false;
	    }
	    writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

	    // apply new VCSEL period
	    writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

	    uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

	    writeReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

	    uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

	    writeReg(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

	    // set_sequence_step_timeout() end
	  }
	  else if (type == VcselPeriodFinalRange)
	  {
	    switch (period_pclks)
	    {
	      case 8:
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
	        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
	        writeReg(0xFF, 0x01);
	        writeReg(ALGO_PHASECAL_LIM, 0x30);
	        writeReg(0xFF, 0x00);
	        break;

	      case 10:
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
	        writeReg(0xFF, 0x01);
	        writeReg(ALGO_PHASECAL_LIM, 0x20);
	        writeReg(0xFF, 0x00);
	        break;

	      case 12:
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
	        writeReg(0xFF, 0x01);
	        writeReg(ALGO_PHASECAL_LIM, 0x20);
	        writeReg(0xFF, 0x00);
	        break;

	      case 14:
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
	        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
	        writeReg(0xFF, 0x01);
	        writeReg(ALGO_PHASECAL_LIM, 0x20);
	        writeReg(0xFF, 0x00);
	        break;

	      default:
	        // invalid period
	        return false;
	    }

	    // apply new VCSEL period
	    writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	    // "For the final range timeout, the pre-range timeout
	    //  must be added. To do this both final and pre-range
	    //  timeouts must be expressed in macro periods MClks
	    //  because they have different vcsel periods."

	    uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

	    if (enables.pre_range){
	      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
	    }

	    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));

	    // set_sequence_step_timeout end
	  }
	  else
	  {
	    // invalid type
	    return false;
	  }

	  // "Finally, the timing budget must be re-applied"

	  setMeasurementTimingBudget(measurement_timing_budget_us);

	  // "Perform the phase calibration. This is needed after changing on vcsel period."
	  // VL53L0X_perform_phase_calibration() begin

	  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
	  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
	  performSingleRefCalibration(0x0);
	  writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

	  // VL53L0X_perform_phase_calibration() end

	  return true;
}


uint32_t VL53L0X::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
	return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
	return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X::decodeTimeout(uint16_t reg_val) {
	// format: "(LSByte * 2^MSByte) + 1"
	return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t VL53L0X::encodeTimeout(uint16_t timeout_mclks) {
	// format: "(LSByte * 2^MSByte) + 1"

	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);
	}
	else { return 0; }
}

// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte) {
	writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

	startTimeout();
	while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (checkTimeoutExpired()) { return false; }
	}

	writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	writeReg(SYSRANGE_START, 0x00);

	return true;
}

int
VL53L0X::probe()
{
	return measure();
}

void
VL53L0X::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
VL53L0X::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
VL53L0X::get_minimum_distance()
{
	return _min_distance;
}

float
VL53L0X::get_maximum_distance()
{
	return _max_distance;
}

int
VL53L0X::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
		case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
				/* do we need to start internal polling? */
				bool want_start = (_measure_ticks == 0);

				/* set interval for next measurement to minimum legal value */
				_measure_ticks = USEC2TICK(_conversion_interval);

				/* if we need to start the poll state machine, do it */
				if (want_start) {
					start();
				}
				return OK;
			}
			/* adjust to a legal polling interval in Hz */
			default: {
				/* do we need to start internal polling? */
				bool want_start = (_measure_ticks == 0);

				/* convert hz to tick interval via microseconds */
				int ticks = USEC2TICK(1000000 / arg);

				/* check against maximum rate */
				if (ticks < USEC2TICK(_conversion_interval)) {
					return -EINVAL;
				}

				/* update interval for next measurement */
				_measure_ticks = ticks;

				/* if we need to start the poll state machine, do it */
				if (want_start) {
					start();
				}
				return OK;
			}
		}
	}
	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}
		return (1000 / _measure_ticks);
	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100)) {
			return -EINVAL;
		}

		irqstate_t flags = px4_enter_critical_section();

		if (!_reports->resize(arg)) {
			px4_leave_critical_section(flags);
			return -ENOMEM;
		}

		px4_leave_critical_section(flags);

		return OK;
	}
	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case RANGEFINDERIOCSETMINIUMDISTANCE: {
		set_minimum_distance(*(float *)arg);
		return 0;
	}
	break;

	case RANGEFINDERIOCSETMAXIUMDISTANCE: {
		set_maximum_distance(*(float *)arg);
		return 0;
	}
	break;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
VL53L0X::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

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

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
VL53L0X::measure()
{
	/*
	 * this is stub, at least for now
	 */

	return OK;
}

int
VL53L0X::collect()
{
	int	ret = -EIO;

	writeReg(SYSRANGE_START, 0x01);

	uint16_t distance_cm = (float)readReg16(RESULT_RANGE_STATUS + 10) / 10.0f;
	float distance_m = float(distance_cm) * 1e-2f;

	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = 8;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
VL53L0X::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}

void
VL53L0X::stop()
{
	work_cancel(HPWORK, &_work);
}

void
VL53L0X::cycle_trampoline(void *arg)
{
	VL53L0X *dev = (VL53L0X *)arg;
	if(!dev)
		return;
	dev->cycle();
}

void
VL53L0X::cycle()
{
	/* Collect results */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&VL53L0X::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));
}

void
VL53L0X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace vl53l0x
{

VL53L0X	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd = -1;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new VL53L0X(VL53L0X_BUS);

	if (g_dev == nullptr) {
		printf("Cannot create device\n");
		goto fail;
	}

	if (OK != g_dev->init()) {
		printf("Cannot init device\n");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		printf("Cannot open device\n");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		printf("Cannot ioctl device\n");
		::close(fd);
		goto fail;
	}

	::close(fd);
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	} else {
		errx(1, "driver not running");
	}
	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'vl53l0x start' if the driver is not running", VL53L0X_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("time:        %llu", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("valid %u", (float)report.current_distance > report.min_distance && (float)report.current_distance < report.max_distance ? 1 : 0);
		warnx("measurement: %0.3f m", (double)report.current_distance);
		warnx("time:        %llu", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	::close(fd);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	::close(fd);
	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */

int
vl53l0x_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		vl53l0x::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		vl53l0x::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		vl53l0x::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		vl53l0x::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		vl53l0x::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
