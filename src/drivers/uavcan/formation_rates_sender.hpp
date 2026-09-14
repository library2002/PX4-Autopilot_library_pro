/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file formation_rates_sender.hpp
 *
 * Chain-wing formation link, master (center body) side.
 *
 * Broadcasts nuaa.formation.ControlInput (DroneCAN, DTID 20040) to the wingtip
 * followers. The message carries the master attitude/thrust setpoints plus the
 * hinge-correction feedforward reference.
 *
 * Only a master broadcasts: the sender stays silent when FORM_FOLLOWER_EN is
 * set, so no role handshaking is required on the bus.
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <nuaa/formation/ControlInput.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <px4_platform_common/module_params.h>


class FormationRatesSender : public ModuleParams
{
public:
	FormationRatesSender(uavcan::INode &node);
	~FormationRatesSender() = default;

	/**
	 * Start the periodic broadcast timer. Timer callbacks are dispatched from
	 * UavcanNode::Run() via uavcan::INode::spinOnce().
	 */
	int init();

private:
	/**
	 * Broadcast rate [Hz]. Single point of tuning: the bus load scales linearly
	 * with this, while the follower attitude loops (a few rad/s) need only a
	 * fraction of it. The timer period is built from microseconds on purpose -
	 * `1000 / MAX_RATE_HZ` would truncate to 3 ms and yield 333 Hz.
	 */
	static constexpr unsigned MAX_RATE_HZ = 300;

	/**
	 * CAN transfer priority (0 = highest, 31 = lowest). This is a control-bus
	 * feedforward, so it uses the actuator-command priority class rather than
	 * the default status-message priority.
	 */
	static constexpr unsigned TRANSFER_PRIORITY = 6;

	/**
	 * Age of vehicle_attitude_setpoint beyond which the link goes quiet [us].
	 * Broadcasting a stale master setpoint is worse than not broadcasting.
	 */
	static constexpr uint64_t SETPOINT_TIMEOUT_US = 500000;

	using TimerCbBinder = uavcan::MethodBinder<FormationRatesSender *,
	      void (FormationRatesSender::*)(const uavcan::TimerEvent &)>;

	void periodic_update(const uavcan::TimerEvent &);

	uavcan::TimerEventForwarder<TimerCbBinder> _timer;
	uavcan::Publisher<nuaa::formation::ControlInput> _control_input_pub;

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORM_FOLLOWER_EN>) _param_form_follower_en,

		// Pilot yaw stick to yaw rate conversion, and the yaw rate limit. These are
		// the fixed-wing attitude controller parameters on purpose: the master
		// converts the stick with exactly the same values locally, so the
		// transmitted yaw command reproduces the master's own result.
		(ParamFloat<px4::params::FW_MAN_YR_MAX>) _param_fw_man_yr_max,
		(ParamFloat<px4::params::FW_Y_RMAX>) _param_fw_y_rmax
	)
};
