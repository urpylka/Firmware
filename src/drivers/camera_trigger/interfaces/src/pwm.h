/**
 * @file pwm.h
 *
 * Interface with cameras via pwm.
 *
 */
#pragma once

#ifdef __PX4_NUTTX

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_log.h>

#include "camera_interface.h"

class CameraInterfacePWM : public CameraInterface
{
public:
	CameraInterfacePWM();
	virtual ~CameraInterfacePWM();

	void trigger(bool trigger_on_true);
        void trigger_video(bool trigger_on_true);

	void info();

private:

        void send_pwm(int pwm_level);
	void setup();

};

#endif /* ifdef __PX4_NUTTX */
