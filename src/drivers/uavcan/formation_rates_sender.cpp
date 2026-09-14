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
 * @file formation_rates_sender.cpp
 */

#include "formation_rates_sender.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>


FormationRatesSender::FormationRatesSender(uavcan::INode &node) :
	ModuleParams(nullptr),
	_timer(node),
	_control_input_pub(node)
{
	_control_input_pub.setPriority(TRANSFER_PRIORITY);
}

int FormationRatesSender::init()
{
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &FormationRatesSender::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromUSec(1000000 / MAX_RATE_HZ));
	}

	return 0;
}

void FormationRatesSender::periodic_update(const uavcan::TimerEvent &)
{
	// Only a master drives the link; a follower must stay silent.
	if (_param_form_follower_en.get() != 0) {
		return;
	}

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	vehicle_attitude_setpoint_s attitude_setpoint{};
	vehicle_rates_setpoint_s rates_setpoint{};

	if (!_vehicle_status_sub.copy(&vehicle_status)
	    || !_vehicle_control_mode_sub.copy(&control_mode)
	    || !_attitude_setpoint_sub.copy(&attitude_setpoint)
	    || !_rates_setpoint_sub.copy(&rates_setpoint)) {
		return;
	}

	// The formation link is defined for fixed-wing vehicles, and for a VTOL
	// while it is transitioning to fixed-wing.
	if (vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	    && !vehicle_status.in_transition_mode) {
		return;
	}

	// Freshness gate: go quiet rather than broadcast a stale setpoint.
	const uint64_t now = hrt_absolute_time();

	if (attitude_setpoint.timestamp == 0
	    || now < attitude_setpoint.timestamp
	    || (now - attitude_setpoint.timestamp) > SETPOINT_TIMEOUT_US) {
		return;
	}

	// Master attitude setpoint, converted to Euler angles.
	const matrix::Quatf q_d(attitude_setpoint.q_d);

	if (!q_d.isAllFinite()) {
		return;
	}

	const matrix::Eulerf euler(q_d);

	// Yaw rate source.
	//
	// flag_control_manual_enabled is deliberately used instead of an explicit
	// nav_state list: it is the very predicate the fixed-wing controllers use to
	// decide whether the pilot's yaw enters vehicle_rates_setpoint.yaw
	// (fw_att_control adds the stick to the attitude-loop output, fw_rate_control
	// drives it directly in ACRO). Selecting the source with the same condition
	// the producer uses keeps the two in sync by construction, and it stays
	// correct when nav_state values are renumbered or new manual modes are added.
	//
	// The distinction cannot be inferred from the value itself: in manual modes
	// vehicle_attitude_setpoint.yaw_sp_move_rate keeps its zero-initialized value,
	// which is a *finite* number and would silently zero the follower yaw command.
	float yaw_rate = 0.f;

	if (control_mode.flag_control_manual_enabled) {
		// Pilot is flying: transmit the pilot's yaw *intent* only.
		//
		// The master's own yaw rate setpoint is the turn coordination feedforward
		// plus the stick. Sharing the sum would make every follower add its own
		// feedforward on top, double counting the turn coordination. Each aircraft
		// must therefore compute its own feedforward (with its own airspeed) and
		// only the shared pilot intent travels over the link. The conversion and
		// the rate limit mirror fw_att_control exactly.
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)
		    && PX4_ISFINITE(manual_control_setpoint.yaw)) {
			const float yaw_rate_max = math::radians(_param_fw_y_rmax.get());
			yaw_rate = math::constrain(manual_control_setpoint.yaw * math::radians(_param_fw_man_yr_max.get()),
						   -yaw_rate_max, yaw_rate_max);
		}

	} else {
		// Autonomous: pass the navigation yaw rate command through.
		//
		// Note that nothing in the fixed-wing stack currently writes this field,
		// so it stays at its zero-initialized value and the followers simply turn
		// by banking. The branch is kept because the field is the documented yaw
		// command entry point of the attitude setpoint (a MAVLink ATTITUDE_TARGET
		// from an external computer does write it), so the channel starts working
		// on its own if a producer appears.
		yaw_rate = attitude_setpoint.yaw_sp_move_rate;

		// A non-finite value means "no yaw command", so it maps to zero. It must
		// NOT fall back to vehicle_rates_setpoint.yaw: on a fixed-wing that field
		// is essentially the turn coordination feedforward (the yaw attitude error
		// is stripped by the attitude loop), and forwarding it would make every
		// follower add its own feedforward on top of it - exactly the double
		// counting that the manual branch above avoids.
		if (!PX4_ISFINITE(yaw_rate)) {
			yaw_rate = 0.f;
		}
	}

	nuaa::formation::ControlInput msg{};

	msg.thrust = math::constrain(attitude_setpoint.thrust_body[0], 0.f, 1.f);
	msg.pitch = euler.theta();
	msg.roll_target = euler.phi();
	msg.yaw = yaw_rate;

	// Hinge-correction feedforward reference: the master *desired* roll rate.
	// Deliberately the setpoint, not the measured body rate - see the DSDL
	// field comment for the stability argument.
	msg.master_roll_signed_err = rates_setpoint.roll;

	msg.flags = nuaa::formation::ControlInput::FLAG_VALID;

	(void)_control_input_pub.broadcast(msg);
}
