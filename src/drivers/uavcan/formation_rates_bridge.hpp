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
 * @file formation_rates_bridge.hpp
 *
 * Chain-wing formation link, follower (wingtip body) side.
 *
 * Receives nuaa.formation.ControlInput (DroneCAN, DTID 20040) from the
 * immediate upstream (the master for the first layer, the station one layer
 * closer to the center otherwise) and combines it with the follower's own
 * attitude to derive how this vehicle should move. The result is published as
 * a local vehicle_attitude_setpoint (attitude mode) together with
 * offboard_control_mode.
 *
 * Only the immediate upstream may command this vehicle: all stations share
 * one logical bus (the left/right branches are tied together at the center
 * body), so without the source filter an outer follower would also receive
 * the master's broadcast.
 *
 * A follower that itself has a downstream neighbour (FORM_LAYER <
 * FORM_LAYERS) additionally relays the command onwards: the assembly base
 * attitude, yaw and thrust are passed through unchanged, this vehicle's own
 * desired roll rate replaces the hinge-correction reference, and the source
 * identifier becomes this vehicle's own. The relay is reception-driven and
 * runs while the upstream is fresh, independently of this vehicle's flight
 * mode.
 *
 * The offboard_control_mode heartbeat additionally runs on a local timer so
 * that a short upstream blackout does not starve it: as long as a valid
 * ControlInput arrived within FORM_LINK_LOSS_T, the heartbeat keeps the
 * OFFBOARD mode alive and the consumer-side uORB semantics hold the last
 * attitude setpoint. Past that window the heartbeat stops and the regular
 * offboard-loss failsafe takes over.
 *
 * The name keeps the historical "rates" wording for continuity with the
 * previous implementation; the follower is driven by an attitude setpoint.
 *
 * NOTE: contrary to the previous implementation this is modelled as a
 * formation-link controller living next to the sender rather than as a
 * sensors/ bridge. The sensor bridge framework is built around per-node sensor
 * channels, device IDs and a single ORB topic; none of that applies to a
 * control setpoint link.
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <nuaa/formation/ControlInput.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#include <px4_platform_common/module_params.h>


class FormationRatesBridge : public ModuleParams
{
public:
	FormationRatesBridge(uavcan::INode &node);
	~FormationRatesBridge() = default;

	/**
	 * Register the ControlInput subscriber. Callbacks are dispatched from
	 * UavcanNode::Run() via uavcan::INode::spinOnce().
	 */
	int init();

private:
	using ControlInputCbBinder = uavcan::MethodBinder<FormationRatesBridge *,
	      void (FormationRatesBridge::*)(const uavcan::ReceivedDataStructure<nuaa::formation::ControlInput> &)>;

	void control_input_sub_cb(const uavcan::ReceivedDataStructure<nuaa::formation::ControlInput> &control_input);

	using TimerCbBinder = uavcan::MethodBinder<FormationRatesBridge *,
	      void (FormationRatesBridge::*)(const uavcan::TimerEvent &)>;

	/**
	 * Keep the offboard heartbeat alive while the formation link is considered
	 * alive (a valid ControlInput within FORM_LINK_LOSS_T). Dispatched from
	 * UavcanNode::Run() via uavcan::INode::spinOnce().
	 */
	void heartbeat_update(const uavcan::TimerEvent &);

	/**
	 * Relay the last accepted upstream command to the next station
	 * downstream. The assembly base attitude, yaw and thrust are passed
	 * through unchanged; this vehicle's own desired roll rate replaces the
	 * hinge-correction reference and the source identifier becomes this
	 * vehicle's own. Reception-driven: goes quiet once the upstream is stale
	 * so that the downstream station's own keep-alive and failsafe take over.
	 */
	void relay_update(const uavcan::TimerEvent &);

	/**
	 * Signed lateral position: +1 for LEFT, -1 for RIGHT, 0 for CENTER.
	 * Mirrors the mirror symmetry of the wingtip hinge kinematics.
	 */
	float formation_side_sign() const;

	/**
	 * Position code for the source identifier: 0 = CENTER, 1 = LEFT,
	 * 2 = RIGHT (mirrors the FORM_POSITION values).
	 */
	uint8_t station_side_code() const;

	/**
	 * Source station identifier of the immediate upstream: 0x00 (the center
	 * body) for the first layer, otherwise (own side, layer - 1).
	 */
	uint8_t expected_upstream_source() const;

	/**
	 * This vehicle's own source identifier: (side << 4) | FORM_LAYER.
	 */
	uint8_t own_source_code() const;

	/**
	 * Offboard heartbeat rate [Hz]. Only matters while the upstream is
	 * silent: on reception the callback already publishes the heartbeat, and
	 * the keep-alive merely needs to stay well below COM_OF_LOSS_T
	 * (default 1 s).
	 */
	static constexpr unsigned HEARTBEAT_RATE_HZ = 20;

	/**
	 * Relay broadcast rate [Hz]. Matches the sender cadence so that the
	 * downstream station sees the same link rate as the upstream one.
	 */
	static constexpr unsigned RELAY_RATE_HZ = 300;

	/**
	 * Age of the last accepted upstream command beyond which the relay goes
	 * quiet [us]. Same freshness posture as the sender: a stale base attitude
	 * must not be forwarded.
	 */
	static constexpr uint64_t RELAY_TIMEOUT_US = 500000;

	/**
	 * CAN transfer priority (0 = highest, 31 = lowest), same control-bus
	 * class as the master sender.
	 */
	static constexpr unsigned TRANSFER_PRIORITY = 6;

	/**
	 * Time of the last valid ControlInput reception, 0 = never received.
	 */
	hrt_abstime _last_control_input_time{0};

	/**
	 * Last accepted upstream command, forwarded by the relay with the base,
	 * yaw and thrust fields passed through unchanged.
	 */
	nuaa::formation::ControlInput _last_control_input{};

	uavcan::TimerEventForwarder<TimerCbBinder> _heartbeat_timer;
	uavcan::TimerEventForwarder<TimerCbBinder> _relay_timer;
	uavcan::Subscriber<nuaa::formation::ControlInput, ControlInputCbBinder> _sub_control_input;
	uavcan::Publisher<nuaa::formation::ControlInput> _control_input_pub;

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORM_FOLLOWER_EN>) _param_form_follower_en,
		(ParamInt<px4::params::FORM_POSITION>) _param_form_position,
		(ParamInt<px4::params::FORM_LAYER>) _param_form_layer,
		(ParamInt<px4::params::FORM_LAYERS>) _param_form_layers,
		(ParamFloat<px4::params::FORM_ROLL_LIM>) _param_form_roll_lim,
		(ParamFloat<px4::params::FORM_HINGE_K>) _param_form_hinge_k,
		(ParamFloat<px4::params::FORM_LINK_LOSS_T>) _param_form_link_loss_t
	)
};
