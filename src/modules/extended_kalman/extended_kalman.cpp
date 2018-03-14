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

#include "extended_kalman.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_module.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>


#include <uORB/topics/parameter_update.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/extended_kalman_pos.h>
#include <uORB/topics/actuator_outputs.h>


int ExtendedKalman::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Section that describes the provided module functionality.

		This is a template for a module running as a task in the background with start/stop/status functionality.

		### Implementation
		Section describing the high-level implementation of this module.

		### Examples
		CLI usage example:
		$ module start -f -p 42

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ExtendedKalman::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int ExtendedKalman::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int ExtendedKalman::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("extended_kalman",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ExtendedKalman *ExtendedKalman::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	ExtendedKalman *instance = new ExtendedKalman(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ExtendedKalman::ExtendedKalman(int example_param, bool example_flag)
	: SuperBlock(nullptr, "MOD"),
	_sys_autostart(this, "SYS_AUTOSTART", false)
{
}

void ExtendedKalman::run()
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	/* subscribe to vehicle_gps_position topic */
	int gps_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));

	/* subscribe to actuator_outputs topic */
	int act_out_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));

	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;
  	bool updated = false;
 	bool first_gps_run = true;
  	struct vehicle_gps_position_s ref_gps;
  	struct map_projection_reference_s mp_ref = {};
	orb_advert_t extended_kalman_pos_pub = nullptr;

	matrix::Matrix<float, 12, 1> xhat;
	xhat.setZero();
	matrix::SquareMatrix<float, 12> P = matrix::eye<float, 12>();
	matrix::SquareMatrix<float, 6> R = matrix::eye<float, 6>();
	matrix::SquareMatrix<float, 12> Q = matrix::eye<float, 12>();
	matrix::Matrix<float, 6, 12> H;
	H.setZero();
	matrix::Matrix<float, 12, 6> HT;
	HT.setZero();
	HT(0,0) = 1;
	HT(1,1) = 1;
	HT(2,2) = 1;
	HT(3,9) = 1;
	HT(4,10) = 1;
	HT(5,11) = 1;

	float Ix = 5*10e-3;
	float Iy = 5*10e-3;
	float Iz = 10*10e-3;
	float g = 9.8;
	float m = 1;

	float tx = 0;
	float ty = 0;
	float tz = 0;
	float ft = 0;

	float dt = 0.2;


	while(!should_exit()) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {
      		/* checking for update from IMU */
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw_imu;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw_imu);
				/*PX4_INFO("Raw Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw_imu.accelerometer_m_s2[0],
					 (double)raw_imu.accelerometer_m_s2[1],
					 (double)raw_imu.accelerometer_m_s2[2]);
        		*/

				struct actuator_outputs_s act_out;

				orb_check(act_out_sub_fd, &updated);

				if(updated) {
					orb_copy(ORB_ID(actuator_outputs), act_out_sub_fd, &act_out);
					update_model_inputs(&act_out, tx, ty, tz, ft);
				}

				struct vehicle_gps_position_s raw_gps;

				orb_check(gps_sub_fd, &updated);

				if (updated) {
					if(first_gps_run) {
						orb_copy(ORB_ID(vehicle_gps_position), gps_sub_fd, &ref_gps);
						map_projection_init_timestamped(&mp_ref, ref_gps.lat*10e-8f, ref_gps.lon*10e-8f, hrt_absolute_time());
						first_gps_run = false;
					}
					else {
						float x = 0;
						float y = 0;
						float altitude = (raw_gps.alt - ref_gps.alt) / 1000.0;
						orb_copy(ORB_ID(vehicle_gps_position), gps_sub_fd, &raw_gps);
						map_projection_project(&mp_ref, raw_gps.lat*10e-8f, raw_gps.lon*10e-8f, &x, &y);

						PX4_INFO("Raw GPS:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
						(double)x,
						(double)y,
						(double)altitude,
						(double)raw_gps.alt);

						/*
							K = P * H' / R;
							xhatdot = xhatdot + K * (z - H * xhat);
							xhat = xhat + xhatdot * dt;
							Pdot = F * P + P * F' + Q - P * H' / R * H * P;
							P = P + Pdot * dt;
						*/

						matrix::Matrix<float, 12, 12> F;
						matrix::Matrix<float, 12, 1> xhatdot;
						matrix::Matrix<float, 6, 1> z;
						F.setZero();
						xhatdot.setZero();
						z.setZero();

						z(3,1) = x;
						z(4,1) = y;
						z(5,1) = altitude;

						F(0,0) = xhat(4,0)*xhat(1,0);
						F(0,1) = xhat(5,0)+xhat(4,0)*xhat(0,0);
						F(0,3) = 1;
						F(0,4) = xhat(0,0)*xhat(1,0);
						F(0,5) = xhat(1,0);
						F(1,0) = -xhat(5,0);
						F(1,4) = 1;
						F(1,5) = xhat(0,0);
						F(2,0) = xhat(4,0);
						F(2,4) = xhat(0,0);
						F(2,5) = 1;
						F(3,4) = ((Iy-Iz)/Ix)*xhat(5,0);
						F(3,5) = ((Iy-Iz)/Ix)*xhat(4,0);
						F(4,3) = ((Iz-Ix)/Iy)*xhat(5,0);
						F(4,5) = ((Iz-Ix)/Iy)*xhat(3,0);
						F(5,3) = ((Ix-Iy)/Iz)*xhat(4,0);
						F(5,4) = ((Ix-Iy)/Iz)*xhat(3,0);
						F(6,1) = -g;
						F(6,4) = -xhat(8,0);
						F(6,5) = xhat(7,0);
						F(6,7) = xhat(5,0);
						F(6,8) = -xhat(4,0);
						F(7,0) = g;
						F(7,3) = xhat(8,0);
						F(7,5) = -xhat(6,0);
						F(7,6) = -xhat(5,0);
						F(7,8) = xhat(3,0);
						F(8,3) = -xhat(7,0);
						F(8,4) = xhat(6,0);
						F(8,6) = xhat(4,0);
						F(8,7) = -xhat(3,0);
						F(9,0) = xhat(8,0)*xhat(2,0)+xhat(7,0)*xhat(1,0);
						F(9,1) = xhat(8,0)+xhat(7,0)*xhat(1,0);
						F(9,2) = xhat(8,0)*xhat(0,0)-xhat(7,0);
						F(9,6) = 1;
						F(9,7) = -xhat(2,0)+xhat(0,0)*xhat(1,0);
						F(9,8) = xhat(0,0)*xhat(2,0)+xhat(1,0);
						F(10,0) = xhat(7,0)*xhat(2,0)*xhat(1,0)-xhat(8,0);
						F(10,1) = xhat(7,0)*xhat(0,0)*xhat(2,0)+xhat(8,0)*xhat(2,0);
						F(10,2) = xhat(7,0)*xhat(0,0)*xhat(1,0)+xhat(8,0)*xhat(1,0)+xhat(6,0);
						F(10,6) = xhat(2,0);
						F(10,7) = 1+xhat(0,0)*xhat(1,0)+xhat(2,0);
						F(10,8) = -xhat(0,0)+xhat(2,0)*xhat(1,0);
						F(11,0) = xhat(7,0);
						F(11,1) = -xhat(6,0);
						F(11,6) = -xhat(1,0);
						F(11,7) = xhat(0,0);
						F(11,8) = 1;

						xhatdot(0,0) = xhat(3,0) + xhat(5,0)*xhat(1,0) + xhat(4,0)*xhat(0,0)*xhat(1,0);
						xhatdot(1,0) = xhat(4,0) - xhat(5,0)*xhat(0,0);
						xhatdot(2,0) = xhat(5,0) + xhat(4,0)*xhat(0,0);
						xhatdot(3,0) = ((Iy-Iz)/Ix)*xhat(5,0)*xhat(4,0) + (tx/Ix);
						xhatdot(4,0) = ((Iz-Ix)/Iy)*xhat(3,0)*xhat(5,0) + (ty/Iy);
						xhatdot(5,0) = ((Ix-Iy)/Iz)*xhat(3,0)*xhat(4,0) + (tz/Iz);
						xhatdot(6,0) = xhat(5,0)*xhat(7,0) - xhat(4,0)*xhat(8,0) - g*xhat(1,0);
						xhatdot(7,0) = xhat(3,0)*xhat(8,0) - xhat(5,0)*xhat(6,0) + g*xhat(0,0);
						xhatdot(8,0) = xhat(4,0)*xhat(6,0) - xhat(3,0)*xhat(7,0) + g - (ft/m);
						xhatdot(9,0) = xhat(8,0)*(xhat(0,0)*xhat(2,0) + xhat(1,0)) - xhat(7,0)*(xhat(2,0) - xhat(0,0)*xhat(1,0)) + xhat(6,0);
						xhatdot(10,0) = xhat(7,0)*(1 + xhat(0,0)*xhat(2,0)*xhat(1,0)) - xhat(8,0)*(xhat(0,0) - xhat(2,0)*xhat(1,0)) + xhat(6,0)*xhat(2,0);
						xhatdot(11,0) = xhat(8,0) - xhat(6,0)*xhat(1,0) + xhat(7,0)*xhat(0,0);

						matrix::Matrix<float, 12, 6> K;
						matrix::Matrix<float, 12, 12> Pdot;
						K = P * HT * matrix::inv(R);
						xhatdot = xhatdot + K * (z - H * xhat);
						xhat = xhat + xhatdot * dt;
						Pdot = F * P + P * F.transpose() + Q - P * HT * inv(R) * H * P;
						P = P + Pdot * dt;

						publish_extended_kalman_pos(extended_kalman_pos_pub, xhat(9,0), xhat(10,0), xhat(11,0));
					}
				}
			}
		}
	}

	PX4_INFO("exiting");
}

void ExtendedKalman::update_model_inputs(struct actuator_outputs_s * act_out, float &tx, float &ty, float &tz, float &ft) {
	/* Convert drone thrust levels to tx, ty, tz and ft */
	// PX4_INFO("Actuator Outputs:\t%8.4f\t%8.4f\t%8.4f\t%8.4f", (double)act_out->output[0], (double)act_out->output[1], (double)act_out->output[2], (double)act_out->output[3]);
	tx = 0;
	ty = 0;
	tz = 0;
	ft = 1e-6*(pow(act_out->output[0], 2) + pow(act_out->output[1], 2) + pow(act_out->output[2], 2) + pow(act_out->output[3], 2));
}

void ExtendedKalman::publish_extended_kalman_pos(orb_advert_t &extended_kalman_pos_pub, float x, float y, float z)
{
	extended_kalman_pos_s extended_kalman_pos = {
		.timestamp = hrt_absolute_time(),
		.x = x,
		.y = y,
		.z = z
	};

	if (extended_kalman_pos_pub == nullptr) {
		extended_kalman_pos_pub = orb_advertise_queue(ORB_ID(extended_kalman_pos), &extended_kalman_pos, 10);
	} else {
		orb_publish(ORB_ID(extended_kalman_pos), extended_kalman_pos_pub, &extended_kalman_pos);
	}
}

void ExtendedKalman::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


int extended_kalman_main(int argc, char *argv[])
{
	return ExtendedKalman::main(argc, argv);
}
