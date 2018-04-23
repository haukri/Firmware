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

#include "exogenous_kalman.h"

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
#include <lib/conversion/rotation.h>

#include <iostream>
#include <random>

#include <uORB/topics/parameter_update.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/exogenous_kalman.h>
#include <uORB/topics/actuator_outputs.h>

#define beta 15.2f

int ExogenousKalman::print_usage(const char *reason)
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

int ExogenousKalman::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int ExogenousKalman::custom_command(int argc, char *argv[])
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


int ExogenousKalman::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("exogenous_kalman",
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

ExogenousKalman *ExogenousKalman::instantiate(int argc, char *argv[])
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

	ExogenousKalman *instance = new ExogenousKalman(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ExogenousKalman::ExogenousKalman(int example_param, bool example_flag)
	: SuperBlock(nullptr, "MOD"),
	_sys_autostart(this, "SYS_AUTOSTART", false)
{
}

void ExogenousKalman::run()
{
	PX4_INFO("asdfasdfasdf!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	/* subscribe to vehicle_gps_position topic */
	int gps_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));

	/* subscribe to actuator_outputs topic */
	int act_out_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));

	int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));

	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 10);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;
  	bool updated = false;
 	bool first_gps_run = true;
  	struct vehicle_gps_position_s ref_gps;
	struct vehicle_attitude_s att;
  	struct map_projection_reference_s mp_ref = {};
	orb_advert_t exogenous_kalman_pub = nullptr;

	matrix::Matrix<float, 12, 1> xhat;
	xhat.setZero();
	matrix::Matrix<float, 12, 12> P;
	matrix::Matrix<float, 12, 12> R_inv;
	matrix::Matrix<float, 12, 12> R;
	matrix::Matrix<float, 12, 12> Q;
	matrix::Matrix<float, 12, 12> I;
	matrix::Matrix<float, 12, 12> H;
	matrix::Matrix<float, 12, 12> HT;
	for(int i = 0; i < 12; i++) {
		R_inv(i,i) = 40;
		R(i,i) = 0.025;
	}

	for(int i = 0; i < 12; i++) {
		Q(i,i) = 0.4;
		P(i,i) = 1;
		I(i,i) = 1;
		H(i,i) = 1;
		HT(i,i) = 1;
	}

	float Ix = 0.04;
	float Iy = 0.04;
	float Iz = 0.1;
	float g = -9.8;
	float m = 1.535;

	// Linear Kalman filter
	matrix::Matrix<float, 12, 1> linear_xhat;
	linear_xhat.setZero();
	matrix::Matrix<float, 12, 12> linear_P;
	for(int i = 0; i < 12; i++) {
		linear_P(i,i) = 1;
	}
	matrix::Matrix<float, 12, 4> linear_B;
	linear_B.setZero();
	linear_B(8,0) = 1/m;
	linear_B(3,1) = 1/Ix;
	linear_B(4,2) = 1/Iy;
	linear_B(5,3) = 1/Iz;

	float tx = 0;
	float ty = 0;
	float tz = 0;
	float ft = 0;

	float roll = 0;
	float pitch = 0;
	float yaw = 0;

	float dt = 0.2;
	float last_kalman_dt = -1;

	bool flying = false;

	std::deque<double> gps_check_vector;
	float last_yaw = 0;
	float mult_yaw = 0;
	float true_yaw = 0;


	matrix::Matrix<float, 4, 1> test;
	matrix::Matrix<float, 12, 12> K_observer;
	matrix::Matrix<float, 12, 12> P_observer;
	P_observer.setZero();
	double observer_beta = 2.3;
	/*
	P_observer(0,0) = 2.4000 * observer_beta;
	P_observer(0,3) = -0.8000 * observer_beta;
	P_observer(1,1) = 2.4000 * observer_beta;
	P_observer(1,4) = -0.8000 * observer_beta;
	P_observer(2,2) = 2.4000 * observer_beta;
	P_observer(2,5) = -0.8000 * observer_beta;
	P_observer(3,0) = -0.8000 * observer_beta;
	P_observer(3,3) = 1.6000 * observer_beta;
	P_observer(4,1) = -0.8000 * observer_beta;
	P_observer(4,4) = 1.6000 * observer_beta;
	P_observer(5,2) = -0.8000 * observer_beta;
	P_observer(5,5) = 1.6000 * observer_beta;
	P_observer(6,6) = 0.8000 * observer_beta;
	P_observer(6,9) = 0.4000 * observer_beta;
	P_observer(7,7) = 0.8000 * observer_beta;
	P_observer(7,10) = 0.4000 * observer_beta;
	P_observer(8,8) = 0.8000 * observer_beta;
	P_observer(8,11) = 0.4000 * observer_beta;
	P_observer(9,6) = 0.4000 * observer_beta;
	P_observer(9,9) = 1.2000 * observer_beta;
	P_observer(10,7) = 0.4000 * observer_beta;
	P_observer(10,10) = 1.2000 * observer_beta;
	P_observer(11,8) = 0.4000 * observer_beta;
	P_observer(11,11) = 1.2000 * observer_beta;
	*/
	P_observer(0,0) = 1.5000 * observer_beta;
	P_observer(0,3) = 0.5000 * observer_beta;
	P_observer(1,1) = 1.5000 * observer_beta;
	P_observer(1,4) = 0.5000 * observer_beta;
	P_observer(2,2) = 1.5000 * observer_beta;
	P_observer(2,5) = 0.5000 * observer_beta;
	P_observer(3,0) = 0.5000 * observer_beta;
	P_observer(3,3) = 0.5000 * observer_beta;
	P_observer(4,1) = 0.5000 * observer_beta;
	P_observer(4,4) = 0.5000 * observer_beta;
	P_observer(5,2) = 0.5000 * observer_beta;
	P_observer(5,5) = 0.5000 * observer_beta;
	P_observer(6,6) = 1.0000 * observer_beta;
	P_observer(6,9) = 1.0000 * observer_beta;
	P_observer(7,7) = 1.0000 * observer_beta;
	P_observer(7,10) = 1.0000 * observer_beta;
	P_observer(8,8) = 1.0000 * observer_beta;
	P_observer(8,11) = 1.0000 * observer_beta;
	P_observer(9,6) = 1.0000 * observer_beta;
	P_observer(9,9) = 2.0000 * observer_beta;
	P_observer(10,7) = 1.0000 * observer_beta;
	P_observer(10,10) = 2.0000 * observer_beta;
	P_observer(11,8) = 1.0000 * observer_beta;
	P_observer(11,11) = 2.0000 * observer_beta;
	K_observer = P_observer*HT;

	std::default_random_engine generator;
    std::normal_distribution<float> dist(0.0, 0.02);

	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion for Magwick's filter

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

				// Read and scale gyroscope, accelerometer and magnetometer data

				if(last_kalman_dt < 0) {
					dt = 0.01;
				}
				else {
					long now = hrt_absolute_time();
					dt = (now - last_kalman_dt)/1000000.0;
					last_kalman_dt = now;
					if(dt > 0.05) {
						std::cout << dt << std::endl;
						dt = 0.01;
					}
					// std::cout << dt << std::endl;
				}
				/*
				process_IMU_data(&raw_imu, q, (float)dt);
				yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
				pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
				roll  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
				*/
				
				orb_copy(ORB_ID(vehicle_attitude), attitude_sub_fd, &att);
				
				q[0] = att.q[0];
				q[1] = att.q[1];
				q[2] = att.q[2];
				q[3] = att.q[3];

				yaw   = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
				pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
				roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);	
				
				/*
				roll += 3.14159f;
				if(roll > 3.14159f) {
					roll = 2.0f*3.14159f - roll;
				}
				*/

				if(yaw - last_yaw < -4) {
					mult_yaw += 1;
				}
				else if(yaw - last_yaw > 4) {
					mult_yaw += 1;
				}
				last_yaw = yaw;
				
				roll += dist(generator);
				pitch += dist(generator);
				yaw += dist(generator);
				
				true_yaw = yaw + 3.1415*mult_yaw;

				/*exogenous_kalman_s exogenous_kalman = {
					.timestamp = hrt_absolute_time(),
					.q[0] = roll,
					.q[1] = pitch,
					.q[2] = yaw
				};

				if (exogenous_kalman_pub == nullptr) {
					exogenous_kalman_pub = orb_advertise_queue(ORB_ID(exogenous_kalman), &exogenous_kalman, 10);
				} else {
					orb_publish(ORB_ID(exogenous_kalman), exogenous_kalman_pub, &exogenous_kalman);
				}*/

				/*PX4_INFO("Quaternions:\t%8.4f\t%8.4f\t%8.4f",
						(double)roll,
						(double)pitch,
						(double)yaw);*/
				
				struct actuator_outputs_s act_out;
				orb_check(act_out_sub_fd, &updated);

				if(updated) {
					orb_copy(ORB_ID(actuator_outputs), act_out_sub_fd, &act_out);
					update_model_inputs(&act_out, tx, ty, tz, ft);
					flying = true;
				}

				struct vehicle_gps_position_s raw_gps;

				orb_check(gps_sub_fd, &updated);

				if (true) {
					if(first_gps_run) {
						orb_copy(ORB_ID(vehicle_gps_position), gps_sub_fd, &ref_gps);
						if(gps_check_vector.size() < 6) {
							gps_check_vector.push_back(ref_gps.alt);
						}
						else if(getVariance(gps_check_vector) > 1) {
							gps_check_vector.push_back(ref_gps.alt);
							gps_check_vector.pop_front();
						}
						else {
							PX4_INFO("GPS Check success");
							map_projection_init_timestamped(&mp_ref, ref_gps.lat*10e-8f, ref_gps.lon*10e-8f, hrt_absolute_time());
							first_gps_run = false;
						}
					}
					else if(flying) {
						float x = 0;
						float y = 0;
						orb_copy(ORB_ID(vehicle_gps_position), gps_sub_fd, &raw_gps);
						map_projection_project(&mp_ref, raw_gps.lat*10e-8f, raw_gps.lon*10e-8f, &x, &y);
						float altitude = -(raw_gps.alt - ref_gps.alt) / 1000.0;

						/*
							K = P * H' / R;
							xhatdot = xhatdot + K * (z - H * xhat);
							xhat = xhat + xhatdot * dt;
							Pdot = F * P + P * F' + Q - P * H' / R * H * P;
							P = P + Pdot * dt;
						*/

						matrix::Matrix<float, 12, 12> F;
						matrix::Matrix<float, 12, 1> xhatdot, xhat_t, dyt, dym;
						matrix::Matrix<float, 12, 1> z;
						F.setZero();
						xhatdot.setZero();
						z.setZero();

						z(0,0) = roll;
						z(1,0) = pitch;
						z(2,0) = true_yaw;
						z(3,0) = raw_imu.gyro_rad[0];
						z(4,0) = -raw_imu.gyro_rad[1];
						z(5,0) = -raw_imu.gyro_rad[2];
						z(6,0) = raw_gps.vel_n_m_s;
						z(7,0) = raw_gps.vel_e_m_s;
						z(8,0) = raw_gps.vel_d_m_s;
						z(9,0) = x;
						z(10,0) = y;
						z(11,0) = altitude;

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
						xhatdot = xhatdot + (K_observer * (z - H * xhat));
						// Step 1
						xhat_t = xhat + (xhatdot * (dt/2));
						// Step 2
						dyt(0,0) = xhat_t(3,0) + xhat_t(5,0)*xhat_t(1,0) + xhat_t(4,0)*xhat_t(0,0)*xhat_t(1,0);
						dyt(1,0) = xhat_t(4,0) - xhat_t(5,0)*xhat_t(0,0);
						dyt(2,0) = xhat_t(5,0) + xhat_t(4,0)*xhat_t(0,0);
						dyt(3,0) = ((Iy-Iz)/Ix)*xhat_t(5,0)*xhat_t(4,0) + (tx/Ix);
						dyt(4,0) = ((Iz-Ix)/Iy)*xhat_t(3,0)*xhat_t(5,0) + (ty/Iy);
						dyt(5,0) = ((Ix-Iy)/Iz)*xhat_t(3,0)*xhat_t(4,0) + (tz/Iz);
						dyt(6,0) = xhat_t(5,0)*xhat_t(7,0) - xhat_t(4,0)*xhat_t(8,0) - g*xhat_t(1,0);
						dyt(7,0) = xhat_t(3,0)*xhat_t(8,0) - xhat_t(5,0)*xhat_t(6,0) + g*xhat_t(0,0);
						dyt(8,0) = xhat_t(4,0)*xhat_t(6,0) - xhat_t(3,0)*xhat_t(7,0) + g - (ft/m);
						dyt(9,0) = xhat_t(8,0)*(xhat_t(0,0)*xhat_t(2,0) + xhat_t(1,0)) - xhat_t(7,0)*(xhat_t(2,0) - xhat_t(0,0)*xhat_t(1,0)) + xhat_t(6,0);
						dyt(10,0) = xhat_t(7,0)*(1 + xhat_t(0,0)*xhat_t(2,0)*xhat_t(1,0)) - xhat_t(8,0)*(xhat_t(0,0) - xhat_t(2,0)*xhat_t(1,0)) + xhat_t(6,0)*xhat_t(2,0);
						dyt(11,0) = xhat_t(8,0) - xhat_t(6,0)*xhat_t(1,0) + xhat_t(7,0)*xhat_t(0,0);
						dyt = dyt + K_observer * (z - H*xhat_t);
						xhat_t = xhat + dyt*(dt/2);
						// Step 3
						dym(0,0) = xhat_t(3,0) + xhat_t(5,0)*xhat_t(1,0) + xhat_t(4,0)*xhat_t(0,0)*xhat_t(1,0);
						dym(1,0) = xhat_t(4,0) - xhat_t(5,0)*xhat_t(0,0);
						dym(2,0) = xhat_t(5,0) + xhat_t(4,0)*xhat_t(0,0);
						dym(3,0) = ((Iy-Iz)/Ix)*xhat_t(5,0)*xhat_t(4,0) + (tx/Ix);
						dym(4,0) = ((Iz-Ix)/Iy)*xhat_t(3,0)*xhat_t(5,0) + (ty/Iy);
						dym(5,0) = ((Ix-Iy)/Iz)*xhat_t(3,0)*xhat_t(4,0) + (tz/Iz);
						dym(6,0) = xhat_t(5,0)*xhat_t(7,0) - xhat_t(4,0)*xhat_t(8,0) - g*xhat_t(1,0);
						dym(7,0) = xhat_t(3,0)*xhat_t(8,0) - xhat_t(5,0)*xhat_t(6,0) + g*xhat_t(0,0);
						dym(8,0) = xhat_t(4,0)*xhat_t(6,0) - xhat_t(3,0)*xhat_t(7,0) + g - (ft/m);
						dym(9,0) = xhat_t(8,0)*(xhat_t(0,0)*xhat_t(2,0) + xhat_t(1,0)) - xhat_t(7,0)*(xhat_t(2,0) - xhat_t(0,0)*xhat_t(1,0)) + xhat_t(6,0);
						dym(10,0) = xhat_t(7,0)*(1 + xhat_t(0,0)*xhat_t(2,0)*xhat_t(1,0)) - xhat_t(8,0)*(xhat_t(0,0) - xhat_t(2,0)*xhat_t(1,0)) + xhat_t(6,0)*xhat_t(2,0);
						dym(11,0) = xhat_t(8,0) - xhat_t(6,0)*xhat_t(1,0) + xhat_t(7,0)*xhat_t(0,0);
						dym = dym + K_observer * (z - H*xhat_t);
						xhat_t = xhat + dym*dt;
						dym += dyt;
						// Step 4
						dyt(0,0) = xhat_t(3,0) + xhat_t(5,0)*xhat_t(1,0) + xhat_t(4,0)*xhat_t(0,0)*xhat_t(1,0);
						dyt(1,0) = xhat_t(4,0) - xhat_t(5,0)*xhat_t(0,0);
						dyt(2,0) = xhat_t(5,0) + xhat_t(4,0)*xhat_t(0,0);
						dyt(3,0) = ((Iy-Iz)/Ix)*xhat_t(5,0)*xhat_t(4,0) + (tx/Ix);
						dyt(4,0) = ((Iz-Ix)/Iy)*xhat_t(3,0)*xhat_t(5,0) + (ty/Iy);
						dyt(5,0) = ((Ix-Iy)/Iz)*xhat_t(3,0)*xhat_t(4,0) + (tz/Iz);
						dyt(6,0) = xhat_t(5,0)*xhat_t(7,0) - xhat_t(4,0)*xhat_t(8,0) - g*xhat_t(1,0);
						dyt(7,0) = xhat_t(3,0)*xhat_t(8,0) - xhat_t(5,0)*xhat_t(6,0) + g*xhat_t(0,0);
						dyt(8,0) = xhat_t(4,0)*xhat_t(6,0) - xhat_t(3,0)*xhat_t(7,0) + g - (ft/m);
						dyt(9,0) = xhat_t(8,0)*(xhat_t(0,0)*xhat_t(2,0) + xhat_t(1,0)) - xhat_t(7,0)*(xhat_t(2,0) - xhat_t(0,0)*xhat_t(1,0)) + xhat_t(6,0);
						dyt(10,0) = xhat_t(7,0)*(1 + xhat_t(0,0)*xhat_t(2,0)*xhat_t(1,0)) - xhat_t(8,0)*(xhat_t(0,0) - xhat_t(2,0)*xhat_t(1,0)) + xhat_t(6,0)*xhat_t(2,0);
						dyt(11,0) = xhat_t(8,0) - xhat_t(6,0)*xhat_t(1,0) + xhat_t(7,0)*xhat_t(0,0);
						dyt = dyt + K_observer * (z - H*xhat_t);

						xhat = xhat + (xhatdot + dyt + dym*2)*(dt/6);


						if(xhat(11,0) > 0) {
							xhat(11,0) = 0;
						}

						// Linear Kalman filter
						
						F(0,3) = 1;
						F(1,4) = 1;
						F(2,5) = 1;
						F(9,6) = 1;
						F(10,7) = 1;
						F(11,8) = 1;
						
						/*
						F(0,0) = xhat(4,0)*xhat(1,0); F(0,1) = xhat(5,0)+xhat(4,0)*xhat(0,0); F(0,3) = 1; F(0,4) = xhat(0,0)*xhat(1,0); F(0,5) = xhat(1,0);
						F(1,0) = -xhat(5,0); F(1,4) = 1; F(1,5) = xhat(0,0);
						F(2,0) = xhat(4,0); F(2,4) = xhat(0,0); F(2,5) = 1;
						F(3,4) = ((Iy-Iz)/Ix)*xhat(5,0); F(3,5) = ((Iy-Iz)/Ix)*xhat(4,0);
						F(4,3) = ((Iz-Ix)/Iy)*xhat(5,0); F(4,5) = ((Iz-Ix)/Iy)*xhat(3,0);
						F(5,3) = ((Ix-Iy)/Iz)*xhat(4,0); F(5,4) = ((Ix-Iy)/Iz)*xhat(3,0);
						F(6,1) = -g; F(6,4) = -xhat(8,0); F(6,5) = xhat(7,0); F(6,7) = xhat(5,0); F(6,8) = -xhat(4,0);
						F(7,0) = g; F(7,3) = xhat(8,0); F(7,5) = -xhat(6,0); F(7,6) = -xhat(5,0); F(7,8) = xhat(3,0);
						F(8,3) = -xhat(7,0); F(8,4) = xhat(6,0); F(8,6) = xhat(4,0); F(8,7) = -xhat(3,0);
						F(9,0) = xhat(8,0)*xhat(2,0)+xhat(7,0)*xhat(1,0); F(9,1) = xhat(8,0)+xhat(7,0)*xhat(1,0); F(9,2) = xhat(8,0)*xhat(0,0)-xhat(7,0); F(9,6) = 1; F(9,7) = -xhat(2,0)+xhat(0,0)*xhat(1,0); F(9,8) = xhat(0,0)*xhat(2,0)+xhat(1,0);
						F(10,0) = xhat(7,0)*xhat(2,0)*xhat(1,0)-xhat(8,0); F(10,1) = xhat(7,0)*xhat(0,0)*xhat(2,0)+xhat(8,0)*xhat(2,0); F(10,2) = xhat(7,0)*xhat(0,0)*xhat(1,0)+xhat(8,0)*xhat(1,0)+xhat(6,0); F(10,6) = xhat(2,0); F(10,7) = 1+xhat(0,0)*xhat(1,0)+xhat(2,0); F(10,8) = -xhat(0,0)+xhat(2,0)*xhat(1,0);
						F(11,0) = xhat(7,0); F(11,1) = -xhat(6,0); F(11,6) = -xhat(1,0); F(11,7) = xhat(0,0); F(11,8) = 1;
						*/
						matrix::Matrix<float, 12, 12> linear_K;
						matrix::Matrix<float, 12, 12> linear_Pdot;
						matrix::Matrix<float, 12, 1> linear_xhatdot, linear_xhat_t;
						linear_xhatdot.setZero();

						/*
						linear_xhatdot(0,0) = xhat(3,0);
						linear_xhatdot(1,0) = xhat(4,0);
						linear_xhatdot(2,0) = xhat(5,0);
						linear_xhatdot(3,0) = (tx_filtered/Ix);
						linear_xhatdot(4,0) = (ty_filtered/Iy);
						linear_xhatdot(5,0) = (tz_filtered/Iz);
						linear_xhatdot(6,0) = -g*xhat(1,0);
						linear_xhatdot(7,0) = g*xhat(0,0);
						linear_xhatdot(8,0) = g - (ft_filtered/m);
						linear_xhatdot(9,0) = xhat(6,0);
						linear_xhatdot(10,0) = xhat(7,0);
						linear_xhatdot(11,0) = xhat(8,0);
						*/
						
						linear_xhatdot(0,0) = xhat(3,0) + xhat(5,0)*xhat(1,0) + xhat(4,0)*xhat(0,0)*xhat(1,0);
						linear_xhatdot(1,0) = xhat(4,0) - xhat(5,0)*xhat(0,0);
						linear_xhatdot(2,0) = xhat(5,0) + xhat(4,0)*xhat(0,0);
						linear_xhatdot(3,0) = ((Iy-Iz)/Ix)*xhat(5,0)*xhat(4,0) + (tx/Ix);
						linear_xhatdot(4,0) = ((Iz-Ix)/Iy)*xhat(3,0)*xhat(5,0) + (ty/Iy);
						linear_xhatdot(5,0) = ((Ix-Iy)/Iz)*xhat(3,0)*xhat(4,0) + (tz/Iz);
						linear_xhatdot(6,0) = xhat(5,0)*xhat(7,0) - xhat(4,0)*xhat(8,0) - g*xhat(1,0);
						linear_xhatdot(7,0) = xhat(3,0)*xhat(8,0) - xhat(5,0)*xhat(6,0) + g*xhat(0,0);
						linear_xhatdot(8,0) = xhat(4,0)*xhat(6,0) - xhat(3,0)*xhat(7,0) + g - (ft/m);
						linear_xhatdot(9,0) = xhat(8,0)*(xhat(0,0)*xhat(2,0) + xhat(1,0)) - xhat(7,0)*(xhat(2,0) - xhat(0,0)*xhat(1,0)) + xhat(6,0);
						linear_xhatdot(10,0) = xhat(7,0)*(1 + xhat(0,0)*xhat(2,0)*xhat(1,0)) - xhat(8,0)*(xhat(0,0) - xhat(2,0)*xhat(1,0)) + xhat(6,0)*xhat(2,0);
						linear_xhatdot(11,0) = xhat(8,0) - xhat(6,0)*xhat(1,0) + xhat(7,0)*xhat(0,0);
						
						/*
						linear_xhatdot(0,0) = xhat(3,0);
						linear_xhatdot(1,0) = xhat(4,0);
						linear_xhatdot(2,0) = xhat(5,0);
						linear_xhatdot(3,0) = (tx/Ix);
						linear_xhatdot(4,0) = (ty/Iy);
						linear_xhatdot(5,0) = (tz/Iz);
						linear_xhatdot(6,0) = - g*xhat(1,0);
						linear_xhatdot(7,0) = g*xhat(0,0);
						linear_xhatdot(8,0) = g - (ft/m);
						linear_xhatdot(9,0) = xhat(6,0);
						linear_xhatdot(10,0) = xhat(7,0);
						linear_xhatdot(11,0) = xhat(8,0);
						*/
						dt *= 2;
						linear_K = linear_P*HT*R_inv;

						linear_xhatdot = linear_xhatdot + linear_K * (z - H*linear_xhat);
						//linear_xhat = linear_xhat + linear_xhatdot*dt;
						// Step 1
						linear_xhat_t = linear_xhat + linear_xhatdot*(dt/2);
						// Step 2
						
						dyt(0,0) = linear_xhat_t(3,0) + linear_xhat_t(5,0)*linear_xhat_t(1,0) + linear_xhat_t(4,0)*linear_xhat_t(0,0)*linear_xhat_t(1,0);
						dyt(1,0) = linear_xhat_t(4,0) - linear_xhat_t(5,0)*linear_xhat_t(0,0);
						dyt(2,0) = linear_xhat_t(5,0) + linear_xhat_t(4,0)*linear_xhat_t(0,0);
						dyt(3,0) = ((Iy-Iz)/Ix)*linear_xhat_t(5,0)*linear_xhat_t(4,0) + (tx/Ix);
						dyt(4,0) = ((Iz-Ix)/Iy)*linear_xhat_t(3,0)*linear_xhat_t(5,0) + (ty/Iy);
						dyt(5,0) = ((Ix-Iy)/Iz)*linear_xhat_t(3,0)*linear_xhat_t(4,0) + (tz/Iz);
						dyt(6,0) = linear_xhat_t(5,0)*linear_xhat_t(7,0) - linear_xhat_t(4,0)*linear_xhat_t(8,0) - g*linear_xhat_t(1,0);
						dyt(7,0) = linear_xhat_t(3,0)*linear_xhat_t(8,0) - linear_xhat_t(5,0)*linear_xhat_t(6,0) + g*linear_xhat_t(0,0);
						dyt(8,0) = linear_xhat_t(4,0)*linear_xhat_t(6,0) - linear_xhat_t(3,0)*linear_xhat_t(7,0) + g - (ft/m);
						dyt(9,0) = linear_xhat_t(8,0)*(linear_xhat_t(0,0)*linear_xhat_t(2,0) + linear_xhat_t(1,0)) - linear_xhat_t(7,0)*(linear_xhat_t(2,0) - linear_xhat_t(0,0)*linear_xhat_t(1,0)) + linear_xhat_t(6,0);
						dyt(10,0) = linear_xhat_t(7,0)*(1 + linear_xhat_t(0,0)*linear_xhat_t(2,0)*linear_xhat_t(1,0)) - linear_xhat_t(8,0)*(linear_xhat_t(0,0) - linear_xhat_t(2,0)*linear_xhat_t(1,0)) + linear_xhat_t(6,0)*linear_xhat_t(2,0);
						dyt(11,0) = linear_xhat_t(8,0) - linear_xhat_t(6,0)*linear_xhat_t(1,0) + linear_xhat_t(7,0)*linear_xhat_t(0,0);
						/*
						dyt(0,0) = xhat_t(3,0);
						dyt(1,0) = xhat_t(4,0);
						dyt(2,0) = xhat_t(5,0);
						dyt(3,0) = (tx/Ix);
						dyt(4,0) = (ty/Iy);
						dyt(5,0) = (tz/Iz);
						dyt(6,0) = - g*xhat_t(1,0);
						dyt(7,0) = g*xhat_t(0,0);
						dyt(8,0) = g - (ft/m);
						dyt(9,0) = xhat_t(6,0);
						dyt(10,0) = xhat_t(7,0);
						dyt(11,0) = xhat_t(8,0);
						*/
						dyt = dyt + linear_K * (z - H*linear_xhat_t);
						linear_xhat_t = linear_xhat + dyt*(dt/2);
						// Step 3
						
						dym(0,0) = linear_xhat_t(3,0) + linear_xhat_t(5,0)*linear_xhat_t(1,0) + linear_xhat_t(4,0)*linear_xhat_t(0,0)*linear_xhat_t(1,0);
						dym(1,0) = linear_xhat_t(4,0) - linear_xhat_t(5,0)*linear_xhat_t(0,0);
						dym(2,0) = linear_xhat_t(5,0) + linear_xhat_t(4,0)*linear_xhat_t(0,0);
						dym(3,0) = ((Iy-Iz)/Ix)*linear_xhat_t(5,0)*linear_xhat_t(4,0) + (tx/Ix);
						dym(4,0) = ((Iz-Ix)/Iy)*linear_xhat_t(3,0)*linear_xhat_t(5,0) + (ty/Iy);
						dym(5,0) = ((Ix-Iy)/Iz)*linear_xhat_t(3,0)*linear_xhat_t(4,0) + (tz/Iz);
						dym(6,0) = linear_xhat_t(5,0)*linear_xhat_t(7,0) - linear_xhat_t(4,0)*linear_xhat_t(8,0) - g*linear_xhat_t(1,0);
						dym(7,0) = linear_xhat_t(3,0)*linear_xhat_t(8,0) - linear_xhat_t(5,0)*linear_xhat_t(6,0) + g*linear_xhat_t(0,0);
						dym(8,0) = linear_xhat_t(4,0)*linear_xhat_t(6,0) - linear_xhat_t(3,0)*linear_xhat_t(7,0) + g - (ft/m);
						dym(9,0) = linear_xhat_t(8,0)*(linear_xhat_t(0,0)*linear_xhat_t(2,0) + linear_xhat_t(1,0)) - linear_xhat_t(7,0)*(linear_xhat_t(2,0) - linear_xhat_t(0,0)*linear_xhat_t(1,0)) + linear_xhat_t(6,0);
						dym(10,0) = linear_xhat_t(7,0)*(1 + linear_xhat_t(0,0)*linear_xhat_t(2,0)*linear_xhat_t(1,0)) - linear_xhat_t(8,0)*(linear_xhat_t(0,0) - linear_xhat_t(2,0)*linear_xhat_t(1,0)) + linear_xhat_t(6,0)*linear_xhat_t(2,0);
						dym(11,0) = linear_xhat_t(8,0) - linear_xhat_t(6,0)*linear_xhat_t(1,0) + linear_xhat_t(7,0)*linear_xhat_t(0,0);
						/*
						dym(0,0) = xhat_t(3,0);
						dym(1,0) = xhat_t(4,0);
						dym(2,0) = xhat_t(5,0);
						dym(3,0) = (tx/Ix);
						dym(4,0) = (ty/Iy);
						dym(5,0) = (tz/Iz);
						dym(6,0) = - g*xhat_t(1,0);
						dym(7,0) = g*xhat_t(0,0);
						dym(8,0) = g - (ft/m);
						dym(9,0) = xhat_t(6,0);
						dym(10,0) = xhat_t(7,0);
						dym(11,0) = xhat_t(8,0);
						*/
						dym = dym + linear_K * (z - H*linear_xhat_t);
						linear_xhat_t = linear_xhat + dym*dt;
						dym += dyt;
						// Step 4
						
						dyt(0,0) = linear_xhat_t(3,0) + linear_xhat_t(5,0)*linear_xhat_t(1,0) + linear_xhat_t(4,0)*linear_xhat_t(0,0)*linear_xhat_t(1,0);
						dyt(1,0) = linear_xhat_t(4,0) - linear_xhat_t(5,0)*linear_xhat_t(0,0);
						dyt(2,0) = linear_xhat_t(5,0) + linear_xhat_t(4,0)*linear_xhat_t(0,0);
						dyt(3,0) = ((Iy-Iz)/Ix)*linear_xhat_t(5,0)*linear_xhat_t(4,0) + (tx/Ix);
						dyt(4,0) = ((Iz-Ix)/Iy)*linear_xhat_t(3,0)*linear_xhat_t(5,0) + (ty/Iy);
						dyt(5,0) = ((Ix-Iy)/Iz)*linear_xhat_t(3,0)*linear_xhat_t(4,0) + (tz/Iz);
						dyt(6,0) = linear_xhat_t(5,0)*linear_xhat_t(7,0) - linear_xhat_t(4,0)*linear_xhat_t(8,0) - g*linear_xhat_t(1,0);
						dyt(7,0) = linear_xhat_t(3,0)*linear_xhat_t(8,0) - linear_xhat_t(5,0)*linear_xhat_t(6,0) + g*linear_xhat_t(0,0);
						dyt(8,0) = linear_xhat_t(4,0)*linear_xhat_t(6,0) - linear_xhat_t(3,0)*linear_xhat_t(7,0) + g - (ft/m);
						dyt(9,0) = linear_xhat_t(8,0)*(linear_xhat_t(0,0)*linear_xhat_t(2,0) + linear_xhat_t(1,0)) - linear_xhat_t(7,0)*(linear_xhat_t(2,0) - linear_xhat_t(0,0)*linear_xhat_t(1,0)) + linear_xhat_t(6,0);
						dyt(10,0) = linear_xhat_t(7,0)*(1 + linear_xhat_t(0,0)*linear_xhat_t(2,0)*linear_xhat_t(1,0)) - linear_xhat_t(8,0)*(linear_xhat_t(0,0) - linear_xhat_t(2,0)*linear_xhat_t(1,0)) + linear_xhat_t(6,0)*linear_xhat_t(2,0);
						dyt(11,0) = linear_xhat_t(8,0) - linear_xhat_t(6,0)*linear_xhat_t(1,0) + linear_xhat_t(7,0)*linear_xhat_t(0,0);
						/*
						dyt(0,0) = xhat_t(3,0);
						dyt(1,0) = xhat_t(4,0);
						dyt(2,0) = xhat_t(5,0);
						dyt(3,0) = (tx/Ix);
						dyt(4,0) = (ty/Iy);
						dyt(5,0) = (tz/Iz);
						dyt(6,0) = - g*xhat_t(1,0);
						dyt(7,0) = g*xhat_t(0,0);
						dyt(8,0) = g - (ft/m);
						dyt(9,0) = xhat_t(6,0);
						dyt(10,0) = xhat_t(7,0);
						dyt(11,0) = xhat_t(8,0);
						*/
						dyt = dyt + linear_K * (z - H*linear_xhat_t);

						linear_xhat = linear_xhat + (linear_xhatdot + dyt + dym*2)*(dt/6);

						linear_Pdot = F*linear_P + linear_P*F.transpose() + Q - linear_P*HT * R_inv * H * linear_P;
						linear_P = linear_P + linear_Pdot*dt;
						
						
						/*
						// Prediction State
						linear_Pdot = F * linear_P * F.transpose();
						linear_xhatdot = F * linear_xhat + linear_B * linear_u;

						// Corrective State
						S = H * linear_P * HT + R;
						linear_K = linear_P * HT * inv(S);
						linear_xhat = linear_xhatdot + linear_K*(z - H * linear_xhatdot);
						linear_P = (I-linear_K*H)*linear_Pdot*((I-linear_K*H).transpose())+linear_K*R*linear_K.transpose();
						//linear_P = (I-linear_K * H) * linear_Pdot;
						*/

						exogenous_kalman_s exogenous_kalman = {
							.timestamp = hrt_absolute_time(),
							.x = xhat(0,0),
							.y = xhat(1,0),
							.z = xhat(2,0),
							.x_gps = -raw_imu.gyro_rad[1],
							.y_gps = pitch,
							.z_gps = roll,
							.roll = linear_xhat(0,0),
							.pitch = linear_xhat(1,0),
							.yaw = linear_xhat(2,0)
						};

						if (exogenous_kalman_pub == nullptr) {
							exogenous_kalman_pub = orb_advertise_queue(ORB_ID(exogenous_kalman), &exogenous_kalman, 10);
						} else {
							orb_publish(ORB_ID(exogenous_kalman), exogenous_kalman_pub, &exogenous_kalman);
						}

						//publish_exogenous_kalman(exogenous_kalman_pub, xhat(9,0), xhat(10,0), xhat(11,0));
					}
				}
			}
		}
	}

	PX4_INFO("exiting");
}

void ExogenousKalman::update_model_inputs(struct actuator_outputs_s * act_out, float &tx, float &ty, float &tz, float &ft) {
	/* Convert drone thrust levels to tx, ty, tz and ft */
	// PX4_INFO("Actuator Outputs:\t%8.4f\t%8.4f\t%8.4f\t%8.4f", (double)act_out->output[0], (double)act_out->output[1], (double)act_out->output[2], (double)act_out->output[3]);
	float b = 1.3e-6;
	float l = 0.25;
	float d = 5e-8;

	// std::cout << act_out->output[0] << std::endl;
	//PX4_INFO("Act:\t%8.4f\t%8.4f\t%8.4f\t%8.4f", (double)act_out->output[0], (double)act_out->output[1], (double)act_out->output[2], (double)act_out->output[3] );

	ty = b*l*(pow(act_out->output[1], 2) - pow(act_out->output[0], 2));
	tx = b*l*(pow(act_out->output[3], 2) - pow(act_out->output[2], 2));;
	tz = d*(pow(act_out->output[0], 2) + pow(act_out->output[1], 2) - pow(act_out->output[2], 2) - pow(act_out->output[3], 2));
	ft = b*(pow(act_out->output[0], 2) + pow(act_out->output[1], 2) + pow(act_out->output[2], 2) + pow(act_out->output[3], 2));
	// PX4_INFO("Actuator Outputs:\t%8.4f", (double)ft);

	tx_filtered = _kf_tx.updateEstimate(tx);
	ty_filtered = _kf_ty.updateEstimate(ty);
	tz_filtered = _kf_tz.updateEstimate(tz);
	ft_filtered = _kf_ft.updateEstimate(ft);
}

double ExogenousKalman::getVariance(const std::deque<double>& vec) {
    double mean = 0, M2 = 0, variance = 0;

    size_t n = vec.size();
    for(size_t i = 0; i < n; ++i) {
        double delta = vec[i] - mean;
        mean += delta / (i + 1);
        M2 += delta * (vec[i] - mean);
        variance = M2 / (i + 1);
    }

    return variance;
}

void ExogenousKalman::publish_exogenous_kalman(orb_advert_t &exogenous_kalman_pub, float x, float y, float z) //Logger
{
	exogenous_kalman_s exogenous_kalman = {
		.timestamp = hrt_absolute_time(),
		.x = x,
		.y = y,
		.z = z
	};

	if (exogenous_kalman_pub == nullptr) {
		exogenous_kalman_pub = orb_advertise_queue(ORB_ID(exogenous_kalman), &exogenous_kalman, 10);
	} else {
		orb_publish(ORB_ID(exogenous_kalman), exogenous_kalman_pub, &exogenous_kalman);
	}
}

void ExogenousKalman::process_IMU_data(struct sensor_combined_s *raw_imu, float q[], float dt){
	float accel_scaled[3];
	accel_scaled[0] = raw_imu->accelerometer_m_s2[0];
	accel_scaled[1] = raw_imu->accelerometer_m_s2[1];
	accel_scaled[2] = raw_imu->accelerometer_m_s2[2];
	float gyro_scaled[3];
	gyro_scaled[0] = raw_imu->gyro_rad[0];
	gyro_scaled[1] = raw_imu->gyro_rad[1];
	gyro_scaled[2] = raw_imu->gyro_rad[2];
	float mag_scaled[3]; // Lol... prófum bara....
	mag_scaled[0] = raw_imu->magnetometer_ga[0];
	mag_scaled[1] = raw_imu->magnetometer_ga[1];	
	mag_scaled[2] = raw_imu->magnetometer_ga[2];			
	
	// MadgwickQuaternionUpdate(q, accel_scaled[0], accel_scaled[1], accel_scaled[2], gyro_scaled[0], gyro_scaled[1], gyro_scaled[2], mag_scaled[0], mag_scaled[1], mag_scaled[2], dt);   
	MadgwickQuaternionUpdateIMU(q, accel_scaled[0], accel_scaled[1], accel_scaled[2], gyro_scaled[0], gyro_scaled[1], gyro_scaled[2], dt);	
	// roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	// pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	// yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	
	// pitch *= 180.0f / 3.14159f;
	// yaw   *= 180.0f / 3.14159f - 2.0f; // Declination at Odense, Denmark 2 degrees 15/03/2018
	// roll  *= 180.0f / 3.14159f;
}

void ExogenousKalman::MadgwickQuaternionUpdate(float q[], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;	
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm < 0.00001f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm < 0.00001f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void ExtendedKalman::MadgwickQuaternionUpdateIMU(float q[], float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
	float norm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Auxiliary variables to avoid repeated arithmetic
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_4q0 = 4.0f * q0;
	_4q1 = 4.0f * q1;
	_4q2 = 4.0f * q2;
	_8q1 = 8.0f * q1;
	_8q2 = 8.0f * q2;
	q0q0 = q0 * q0;
	q1q1 = q1 * q1;
	q2q2 = q2 * q2;
	q3q3 = q3 * q3;

	// Gradient decent algorithm corrective step
	s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
	s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
	s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
	s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Apply feedback step
	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void ExogenousKalman::parameters_update(int parameter_update_sub, bool force)
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


int exogenous_kalman_main(int argc, char *argv[])
{
	return ExogenousKalman::main(argc, argv);
}
