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
 * @file formation_rates_bridge.cpp
 */

#include "formation_rates_bridge.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>


namespace
{
// FORM_POSITION values, mirroring the parameter definition values.
constexpr int POSITION_CENTER = 0;
constexpr int POSITION_LEFT = 1;
constexpr int POSITION_RIGHT = 2;
} // namespace


FormationRatesBridge::FormationRatesBridge(uavcan::INode &node) :
	ModuleParams(nullptr),
	_sub_control_input(node)
{
}

int FormationRatesBridge::init()
{
	const int res = _sub_control_input.start(ControlInputCbBinder(this, &FormationRatesBridge::control_input_sub_cb));

	if (res < 0) {
		PX4_WARN("ControlInput sub failed %i", res);
		return res;
	}

	return PX4_OK;
}

float FormationRatesBridge::side_sign() const
{
	switch (_param_form_position.get()) {
	case POSITION_LEFT:
		return 1.f;

	case POSITION_RIGHT:
		return -1.f;

	case POSITION_CENTER:
	default:
		return 0.f;
	}
}

void FormationRatesBridge::control_input_sub_cb(const uavcan::ReceivedDataStructure<nuaa::formation::ControlInput> &msg)
{
	// Only a follower consumes the master's commands.
	if (_param_form_follower_en.get() == 0) {
		return;
	}

	// Refresh the parameters so that the gains can be tuned without a reboot.
	// This class is not part of the UavcanNode parameter tree, hence the
	// explicit subscription.
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	if ((msg.flags & nuaa::formation::ControlInput::FLAG_VALID) == 0) {
		return;
	}

	// Offboard heartbeat. Published even outside OFFBOARD so that the mode can
	// be engaged; the attitude setpoint below stays gated on the mode.
	offboard_control_mode_s offboard_control_mode{};
	offboard_control_mode.timestamp = hrt_absolute_time();
	offboard_control_mode.attitude = true;
	offboard_control_mode.body_rate = false;
	_offboard_control_mode_pub.publish(offboard_control_mode);

	// Discard the command unless this vehicle is actually in OFFBOARD.
	vehicle_status_s vehicle_status{};

	if (!_vehicle_status_sub.copy(&vehicle_status)
	    || vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		return;
	}

	// The follower holds its own heading; the master contributes only the yaw
	// rate command, the heading itself is constrained by the hinge.
	vehicle_attitude_s attitude{};

	if (!_vehicle_attitude_sub.copy(&attitude)) {
		return;
	}

	const matrix::Quatf q(attitude.q);

	if (!q.isAllFinite()) {
		return;
	}

	const float self_yaw = matrix::Eulerf(q).psi();

	const float side = side_sign();
	const float limit = math::radians(_param_form_roll_lim.get());

	// Roll setpoint: follow the master's roll attitude, limited.
	const float roll_sp = math::constrain(msg.roll_target, -limit, limit);

	// Pitch setpoint: master pitch attitude plus the hinge correction. The
	// correction couples the follower pitch to the master's *desired* roll rate
	// and is mirrored between sides by side_sign. The reference is the setpoint
	// rather than the measured rate on purpose - see the DSDL field comment.
	const float pitch_sp = math::constrain(msg.pitch + side * _param_form_hinge_k.get() * msg.master_roll_signed_err,
					       -limit, limit);

	vehicle_attitude_setpoint_s attitude_setpoint{};
	attitude_setpoint.timestamp = hrt_absolute_time();
	const matrix::Quatf q_d(matrix::Eulerf(roll_sp, pitch_sp, self_yaw));
	q_d.copyTo(attitude_setpoint.q_d);
	attitude_setpoint.yaw_sp_move_rate = msg.yaw;
	attitude_setpoint.thrust_body[0] = msg.thrust;
	attitude_setpoint.thrust_body[1] = 0.f;
	attitude_setpoint.thrust_body[2] = 0.f;
	_attitude_setpoint_pub.publish(attitude_setpoint);
}
