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
 * Receives nuaa.formation.ControlInput (DroneCAN, DTID 20040) from the master
 * and combines it with the follower's own attitude to derive how this vehicle
 * should move. The result is published as a local vehicle_attitude_setpoint
 * (attitude mode) together with offboard_control_mode.
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

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
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

	void control_input_sub_cb(const uavcan::ReceivedDataStructure<nuaa::formation::ControlInput> &msg);

	/**
	 * Signed lateral position: +1 for LEFT, -1 for RIGHT, 0 for CENTER.
	 * Mirrors the mirror symmetry of the wingtip hinge kinematics.
	 */
	float side_sign() const;

	uavcan::Subscriber<nuaa::formation::ControlInput, ControlInputCbBinder> _sub_control_input;

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORM_FOLLOWER_EN>) _param_form_follower_en,
		(ParamInt<px4::params::FORM_POSITION>) _param_form_position,
		(ParamFloat<px4::params::FORM_ROLL_LIM>) _param_form_roll_lim,
		(ParamFloat<px4::params::FORM_HINGE_K>) _param_form_hinge_k
	)
};
