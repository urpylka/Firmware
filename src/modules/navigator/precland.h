/***************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file precland.h
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>
#include <px4_module_params.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/vehicle_local_position.h>

#include "navigator_mode.h"
#include "mission_block.h"

enum class PrecLandState {
	Start,			// Starting state
	HorizontalApproach,	// Positioning over landing target while maintaining altitude
	DescendAboveTarget,	// Stay over landing target while descending
	FinalApproach,		// Final landing approach, even without landing target
	Search,			// Search for landing target
	ActiveSearchReset,	// Reset active search attempts counter
	ActiveSearchStart,	// Start active search
	ActiveSearchNewCircle,	// Move the vehicle to the start position in a circle
	ActiveSearch,		// Perform an active search around the circle
	ActiveSearchReturn,	// Return to the initial search position
	Fallback,		// Fallback landing method
	Done			// Done landing
};

enum class PrecLandMode {
	Opportunistic = 1,	// only do precision landing if landing target visible at the beginning
	Required = 2		// try to find landing target if not visible at the beginning
};

class PrecLand : public MissionBlock, public ModuleParams
{
public:
	PrecLand(Navigator *navigator);
	~PrecLand() override = default;

	void on_activation() override;
	void on_active() override;

	void set_mode(PrecLandMode mode) { _mode = mode; };

	PrecLandMode get_mode() { return _mode; };

private:

	void updateParams() override;

	// run the control loop for each state
	void run_state_start();
	void run_state_horizontal_approach();
	void run_state_descend_above_target();
	void run_state_final_approach();
	void run_state_search();
	void run_state_asearch_reset();
	void run_state_asearch_start();
	void run_state_asearch_new_circle();
	void run_state_asearch();
	void run_state_asearch_return();
	void run_state_fallback();

	bool check_hacc_rad(vehicle_local_position_s *vehicle_local_position);
	bool check_setpoint_reached(struct map_projection_reference_s *ref, float target_x, float target_y, float acc_rad);

	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool switch_to_state_start();
	bool switch_to_state_horizontal_approach();
	bool switch_to_state_descend_above_target();
	bool switch_to_state_final_approach();
	bool switch_to_state_search();
	bool switch_to_state_asearch_reset();
	bool switch_to_state_asearch_start();
	bool switch_to_state_asearch_new_circle();
	bool switch_to_state_asearch();
	bool switch_to_state_asearch_return();
	bool switch_to_state_fallback();
	bool switch_to_state_done();

	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);
	void slewrate(float &sp_x, float &sp_y);

	landing_target_pose_s _target_pose{}; /**< precision landing target position */

	int _target_pose_sub{-1};
	bool _target_pose_valid{false}; /**< whether we have received a landing target position message */
	bool _target_pose_updated{false}; /**< wether the landing target position message is updated */

	struct map_projection_reference_s _map_ref {}; /**< reference for local/global projections */

	uint64_t _state_start_time{0}; /**< time when we entered current state */
	uint64_t _last_slewrate_time{0}; /**< time when we last limited setpoint changes */
	uint64_t _target_acquired_time{0}; /**< time when we first saw the landing target during search */
	uint64_t _point_reached_time{0}; /**< time when we reached a setpoint */

	int _search_cnt{0}; /**< counter of how many times we had to search for the landing target */
	float _approach_alt{0.0f}; /**< altitude at which to stay during horizontal approach */

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;

	PrecLandState _state{PrecLandState::Start};

	PrecLandMode _mode{PrecLandMode::Opportunistic};

	float _strict_funnel_k; /** A linear funnel part slope (0 to use cylinder) */
	float _strict_funnel_r_o; /** A linear funnel part offset */

	bool _strict_stop; /** Strict precland stop flag */

	/* Precland fallback actions */
	enum {
		PLD_FOAC_LAND = 0,
		PLD_FOAC_RTL = 1,
		PLD_FOAC_SAFETY_POINT = 2
	};

	float _asearch_radius{0}; /**< current concentric circle radius */
	struct map_projection_reference_s _asearch_ref; /**< reference to the global center of Search state */
	float _asearch_target_x{NAN}; /**< target x to reach (NED, from the center of Search state) */
	float _asearch_target_y{NAN}; /**< target y to reach (NED, from the center of Search state) */
	float _asearch_phi{0}; /**< current phi for the polar coordinate system */
	/**< phi step for the polar coordinate system (calculates for every concentric circle to provide a constant velocity) */
	float _asearch_phi_step{0};
	int _asearch_cnt{0}; /**< counter of how many times we had to search for the landing target actively */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PLD_BTOUT>) _param_pld_btout,
		(ParamFloat<px4::params::PLD_HACC_RAD>) _param_pld_hacc_rad,
		(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_pld_fappr_alt,
		(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_pld_srch_alt,
		(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_pld_srch_tout,
		(ParamInt<px4::params::PLD_MAX_SRCH>) _param_pld_max_srch,
		(ParamBool<px4::params::PLD_STRICT>) _param_pld_strict,
		(ParamFloat<px4::params::PLD_FUNNEL_TR>) _param_pld_funnel_top_rad,
		(ParamFloat<px4::params::PLD_FUNNEL_LEA>) _param_pld_funnel_le_alt,
		(ParamFloat<px4::params::PLD_STATE_TIME>) _param_pld_state_timeout,
		(ParamBool<px4::params::PLD_INFO>) _param_pld_info,
		(ParamInt<px4::params::PLD_FOAC>) _param_pld_fallback_action,
		(ParamBool<px4::params::PLD_ASEN>) _param_pld_asearch_enabled,
		(ParamFloat<px4::params::PLD_ASLR>) _param_pld_asearch_limiting_radius,
		(ParamFloat<px4::params::PLD_ASAR>) _param_pld_asearch_acc_rad,
		(ParamFloat<px4::params::PLD_ASCS>) _param_pld_asearch_cc_step,
		(ParamFloat<px4::params::PLD_ASPS>) _param_pld_asearch_setpoint_step,
		(ParamInt<px4::params::PLD_MAX_ASRCH>) _param_pld_max_asearches
	)

	// non-navigator parameters
	param_t	_handle_param_acceleration_hor{PARAM_INVALID};
	param_t	_handle_param_xy_vel_cruise{PARAM_INVALID};
	float	_param_acceleration_hor{0.0f};
	float	_param_xy_vel_cruise{0.0f};

};
