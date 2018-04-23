/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_module.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include "matrix/Matrix.hpp"
#include <deque>
#include "Kalman.h"

extern "C" __EXPORT int extended_kalman_main(int argc, char *argv[]);


class ExtendedKalman : public ModuleBase<ExtendedKalman>, public control::SuperBlock
{
public:
	ExtendedKalman(int example_param, bool example_flag);

	virtual ~ExtendedKalman() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ExtendedKalman *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);

	void publish_extended_kalman(orb_advert_t &extended_kalman_pub, float x, float y, float z);

	void update_model_inputs(struct actuator_outputs_s *act_out, float &tx, float &ty, float &tz, float &ft);

	double getVariance(const std::deque<double>& vec);
	void MadgwickQuaternionUpdate(float q[], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
	void process_IMU_data(struct sensor_combined_s *raw_imu, float q[], float dt);
	void acc_position_extrapolation(struct sensor_combined_s *raw_imu, float pos_correction[], float velocity[], float position[], float roll, float pitch, float yaw);


	control::BlockParamInt _sys_autostart; /**< example parameter */
	/*********************************/
	/*         Simple Kalman         */
	/*********************************/
	Kalman _kf_tx = Kalman(0.003, 0.001, 0.1);
	Kalman _kf_ty = Kalman(0.003, 0.001, 0.1);
	Kalman _kf_tz = Kalman(0.003, 0.001, 0.1);
	Kalman _kf_ft = Kalman(0.003, 0.001, 0.1);
	Kalman _kf_pitch = Kalman(0.3, 0.1, 0.1);
	float tx_filtered = 0;
	float ty_filtered = 0;
	float tz_filtered = 0;
	float ft_filtered = 0;
	/*********************************/
};

