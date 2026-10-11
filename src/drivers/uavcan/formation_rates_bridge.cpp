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
#include <px4_platform_common/defines.h>

using namespace time_literals;


namespace
{
// FORM_POSITION values, mirroring the parameter definition values.
constexpr int POSITION_CENTER = 0;
constexpr int POSITION_LEFT = 1;
constexpr int POSITION_RIGHT = 2;
} // namespace


FormationRatesBridge::FormationRatesBridge(uavcan::INode &node) :
	ModuleParams(nullptr),
	_heartbeat_timer(node),
	_relay_timer(node),
	_sub_control_input(node),
	_control_input_pub(node)
{
	_control_input_pub.setPriority(TRANSFER_PRIORITY);
}

int FormationRatesBridge::init()
{
	const int res = _sub_control_input.start(ControlInputCbBinder(this, &FormationRatesBridge::control_input_sub_cb));

	if (res < 0) {
		PX4_WARN("ControlInput sub failed %i", res);
		return res;
	}

	if (!_heartbeat_timer.isRunning()) {
		_heartbeat_timer.setCallback(TimerCbBinder(this, &FormationRatesBridge::heartbeat_update));
		_heartbeat_timer.startPeriodic(uavcan::MonotonicDuration::fromUSec(1000000 / HEARTBEAT_RATE_HZ));
	}

	if (!_relay_timer.isRunning()) {
		_relay_timer.setCallback(TimerCbBinder(this, &FormationRatesBridge::relay_update));
		_relay_timer.startPeriodic(uavcan::MonotonicDuration::fromUSec(1000000 / RELAY_RATE_HZ));
	}

	return PX4_OK;
}

float FormationRatesBridge::formation_side_sign() const
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

uint8_t FormationRatesBridge::station_side_code() const
{
	switch (_param_form_position.get()) {
	case POSITION_LEFT:
		return 1;

	case POSITION_RIGHT:
		return 2;

	case POSITION_CENTER:
	default:
		return 0;
	}
}

uint8_t FormationRatesBridge::expected_upstream_source() const
{
	const int layer = _param_form_layer.get();

	// First layer: the immediate upstream is the center body (source 0x00).
	if (layer <= 1) {
		return 0;
	}

	// Deeper layer: the same side, one layer towards the center.
	return static_cast<uint8_t>((station_side_code() << 4) | (layer - 1));
}

uint8_t FormationRatesBridge::own_source_code() const
{
	const int layer = math::constrain(static_cast<int>(_param_form_layer.get()), 0, 15);
	return static_cast<uint8_t>((station_side_code() << 4) | layer);
}

void FormationRatesBridge::control_input_sub_cb(const uavcan::ReceivedDataStructure<nuaa::formation::ControlInput> &control_input)
{
	// Only a follower consumes upstream commands.
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

	if ((control_input.flags & nuaa::formation::ControlInput::FLAG_VALID) == 0) {
		return;
	}

	// Only the immediate upstream may command this vehicle. All stations share
	// one logical bus (the left/right branches are tied together at the center
	// body), so an outer follower physically receives the master's broadcast as
	// well; this filter is what keeps it from acting on - or relaying - the
	// wrong reference.
	if (control_input.source != expected_upstream_source()) {
		return;
	}

	// Mark the link as alive for the timer-driven heartbeat keep-alive and
	// store the command for the relay below. Only accepted (source-filtered)
	// messages count: a stray station must not keep the link alive.
	_last_control_input = control_input;
	_last_control_input_time = hrt_absolute_time();

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

	const float side_sign = formation_side_sign();
	const float limit = math::radians(_param_form_roll_lim.get());

	// Roll setpoint: the assembly base roll attitude (the master's original
	// setpoint, passed through unchanged along the chain), limited.
	const float roll_sp = math::constrain(control_input.base_roll, -limit, limit);

	// Pitch setpoint: the assembly base pitch plus this vehicle's own hinge
	// correction. The correction couples the follower pitch to the *sender's*
	// (immediate upstream's) desired roll rate and is mirrored between sides
	// by side_sign. The reference is the setpoint rather than the measured
	// rate on purpose - see the DSDL field comment.
	const float pitch_sp = math::constrain(control_input.base_pitch + side_sign * _param_form_hinge_k.get() * control_input.roll_rate_ref,
					       -limit, limit);

	vehicle_attitude_setpoint_s attitude_setpoint{};
	attitude_setpoint.timestamp = hrt_absolute_time();
	const matrix::Quatf q_d(matrix::Eulerf(roll_sp, pitch_sp, self_yaw));
	q_d.copyTo(attitude_setpoint.q_d);
	attitude_setpoint.yaw_sp_move_rate = control_input.yaw;
	attitude_setpoint.thrust_body[0] = control_input.thrust;
	attitude_setpoint.thrust_body[1] = 0.f;
	attitude_setpoint.thrust_body[2] = 0.f;
	_attitude_setpoint_pub.publish(attitude_setpoint);
}

void FormationRatesBridge::heartbeat_update(const uavcan::TimerEvent &)
{
	// Only a follower consumes upstream commands; the center body drives the
	// link through FormationRatesSender instead.
	if (_param_form_follower_en.get() == 0) {
		return;
	}

	// Same parameter refresh as the reception callback, so that the timeout
	// can also be tuned while the master is (already) silent.
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	// Keep-alive window: stay quiet once the upstream link is considered lost
	// so that the regular offboard-loss failsafe takes over. The zero
	// timestamp means no valid ControlInput was ever received.
	const uint64_t now = hrt_absolute_time();
	const uint64_t link_timeout_us = static_cast<uint64_t>(_param_form_link_loss_t.get() * 1_s);

	if (_last_control_input_time == 0
	    || now < _last_control_input_time
	    || (now - _last_control_input_time) > link_timeout_us) {
		return;
	}

	// Heartbeat only. The attitude setpoint is deliberately NOT re-published:
	// uORB consumers keep using the last published value, which holds the last
	// upstream command for free.
	offboard_control_mode_s offboard_control_mode{};
	offboard_control_mode.timestamp = hrt_absolute_time();
	offboard_control_mode.attitude = true;
	offboard_control_mode.body_rate = false;
	_offboard_control_mode_pub.publish(offboard_control_mode);
}

void FormationRatesBridge::relay_update(const uavcan::TimerEvent &)
{
	// Only a follower with a downstream neighbour relays: the center body
	// drives the link through FormationRatesSender, and the outermost station
	// of a side stays silent.
	if (_param_form_follower_en.get() == 0
	    || _param_form_layer.get() >= _param_form_layers.get()) {
		return;
	}

	// Same parameter refresh as the other callbacks, so that the layer
	// configuration can be tuned without a reboot.
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	// Reception-driven link: relay while the upstream is alive. Once it goes
	// stale the relay goes quiet and the downstream station's own keep-alive
	// and failsafe take over. The zero timestamp means nothing was ever
	// accepted.
	const uint64_t now = hrt_absolute_time();

	if (_last_control_input_time == 0
	    || now < _last_control_input_time
	    || (now - _last_control_input_time) > RELAY_TIMEOUT_US) {
		return;
	}

	// The correction drive is this vehicle's own desired roll rate (base and
	// correction stay separated down the chain). Without a fresh rate setpoint
	// there is nothing to drive the downstream hinge correction with, so it
	// degrades to zero while the base attitude keeps being relayed.
	float roll_rate_ref = 0.f;
	vehicle_rates_setpoint_s rates_setpoint{};

	if (_vehicle_rates_setpoint_sub.copy(&rates_setpoint)
	    && PX4_ISFINITE(rates_setpoint.roll)
	    && rates_setpoint.timestamp != 0
	    && now >= rates_setpoint.timestamp
	    && (now - rates_setpoint.timestamp) <= RELAY_TIMEOUT_US) {
		roll_rate_ref = rates_setpoint.roll;
	}

	// Base attitude, yaw and thrust are passed through unchanged from the last
	// accepted upstream command; only the correction drive and the source
	// station identify this sender.
	nuaa::formation::ControlInput control_input{};
	control_input.thrust = _last_control_input.thrust;
	control_input.base_pitch = _last_control_input.base_pitch;
	control_input.base_roll = _last_control_input.base_roll;
	control_input.yaw = _last_control_input.yaw;
	control_input.roll_rate_ref = roll_rate_ref;
	control_input.source = own_source_code();
	control_input.flags = nuaa::formation::ControlInput::FLAG_VALID;

	(void)_control_input_pub.broadcast(control_input);
}
