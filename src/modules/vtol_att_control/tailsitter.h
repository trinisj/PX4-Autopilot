/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
* @file tailsitter.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#ifndef TAILSITTER_H
#define TAILSITTER_H

#include "vtol_type.h"

#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/tailsitter_recovery.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>

// [rad] Pitch threshold required for completing transition to fixed-wing in automatic transitions
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW = -1.05f; // -60°

// [rad] Pitch threshold required for completing transition to hover in automatic transitions
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC = -0.26f; // -15°

// [s] Thrust blending duration from fixed-wing to back transition throttle
static constexpr float B_TRANS_THRUST_BLENDING_DURATION = 0.5f;

class Tailsitter : public VtolType
{

public:
	Tailsitter(VtolAttitudeControl *_att_controller);
	~Tailsitter() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;
	void blendThrottleBeginningBackTransition(float scale);

private:
	enum class vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	bool _flag_was_in_trans_mode = false;	// true if mode has just switched to transition

	matrix::Quatf _q_trans_start;
	matrix::Quatf _q_trans_sp;
	matrix::Vector3f _trans_rot_axis;

	/* [2026-09 custom] Front transition tilt progress, advanced only while
	 * the measured sink rate is within VT_F_TR_SINK_MAX. Separate from
	 * _time_since_trans_start (which VtolType keeps advancing regardless -
	 * still used for VT_TRANS_TIMEOUT/VT_TRANS_MIN_TM, unchanged), so the
	 * overall transition timeout budget is untouched; only how fast the
	 * tilt setpoint itself is allowed to advance within that budget
	 * changes. See update_transition_state() and VT_F_TR_SINK_MAX. */
	float _trans_progress_time{0.f};

	/* [2026-09 custom] HANDOFF 9: back-transition analog of _trans_progress_time.
	 * Advances only while measured groundspeed is below VT_B_TR_SPD_GATE - see
	 * update_transition_state() TRANSITION_BACK branch and VT_B_TR_SPD_GATE. */
	float _back_trans_progress_time{0.f};

	/* [2026-09 custom] Measured lift fraction (aerodynamic vertical force /
	 * weight), estimated from the vertical force balance during the front
	 * transition. See updateLiftEstimate(). */
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	float _hover_thrust{NAN};
	float _lift_frac_filt{0.f};
	static constexpr float LIFT_EST_TAU = 0.5f; // [s] low-pass time constant
	void updateLiftEstimate();
	float hoverThrust();

	/* [2026-09 custom] Lift-paced tilt cap: the largest tilt at which the
	 * usable thrust plus the currently measured lift can still hold
	 * altitude. See update_transition_state(). Exposed for logging/debug. */
	float _tilt_cap{0.f};
	float _trans_start_tilt{0.f}; ///< [rad] forward tilt of the transition start attitude

	/* [2026-09 custom] Abort of a front transition that is already fast
	 * (nose near horizontal): go through the ramped back transition instead
	 * of switching to MC instantly. See abortFrontTransitionToBack(). */
	bool _back_trans_from_front{false};
	/* [2026-10 custom] Back transition started by the automatic recovery: start from the ACTUAL nose pitch, never from
	 * fw_virtual_attitude_setpoint. log_4_2026-10-5-20-36-48 / log_1_2026-10-5-20-38-39: when the pitch-up phase ended the FW
	 * controller fell back to TECS, which (climbing at 32 m/s, height-rate setpoint -5 m/s) commanded its lower pitch limit
	 * -30 deg; the back transition adopted that as its start attitude (nose-up 32 deg -> nose-down 30 deg step), dived to
	 * 125 m/s and hit the minimum-altitude quad-chute. */
	bool _back_trans_from_recovery{false};
	bool frontTransitionIsFast();
	void abortFrontTransitionToBack();

	/* [2026-10 custom] Automatic recovery sequence behind ONE transition-to-MC command (VT_REC_EN):
	 *   PITCH_UP -> BACK_TRANSITION -> ATT_HOLD -> ALT_HOLD -> POS_HOLD, see msg/TailsitterRecovery.msg.
	 * This class owns the state machine (it knows the VTOL mode, attitude and velocity); fw_mode_manager and mc_pos_control
	 * only execute their phases from the published tailsitter_recovery message. */
	uORB::Publication<tailsitter_recovery_s> _recovery_pub{ORB_ID(tailsitter_recovery)};
	uint8_t _rec_phase{tailsitter_recovery_s::PHASE_IDLE};
	hrt_abstime _rec_phase_ts{0};      ///< start of the current phase
	hrt_abstime _rec_pub_ts{0};        ///< last publication
	uORB::Subscription _rec_manual_sub{ORB_ID(manual_control_setpoint)};
	uint32_t _rec_mode_sig{0};         ///< flight-mode flags when the recovery started: a change = the pilot switched modes
	uORB::Subscription _rec_status_sub{ORB_ID(vehicle_status)};
	uint8_t _rec_nav_state{0};         ///< nav_state when the recovery started: a change (RC switch, GCS command, failsafe) = takeover
	float _rec_thr_ref{0.f};           ///< throttle stick position when the recovery started (spring-loaded or not)
	bool _rec_cancelled{false};        ///< pilot took over: no new recovery until the next transition command / disarm
	uint32_t recoveryModeSignature() const;
	bool recoveryPilotTakeover(); ///< flight mode (nav_state) changed since the recovery started, or - if VT_REC_STICK > 0 - a stick moved
	void updateRecovery();
	void setRecoveryPhase(uint8_t phase, hrt_abstime now);
	void publishRecovery(hrt_abstime now, bool force);

	void parameters_update() override;

	bool isFrontTransitionCompletedBase() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
					(ParamFloat<px4::params::VT_F_TR_SINK_MAX>) _param_vt_f_tr_sink_max,
					(ParamFloat<px4::params::VT_B_TR_SPD_GATE>) _param_vt_b_tr_spd_gate,
					(ParamFloat<px4::params::VT_F_TR_LIFT_MIN>) _param_vt_f_tr_lift_min,
					(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
					(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
					(ParamFloat<px4::params::MPC_THR_XY_MARG>) _param_mpc_thr_xy_marg,
					(ParamInt<px4::params::VT_REC_EN>) _param_vt_rec_en,
					(ParamFloat<px4::params::VT_REC_PITCH>) _param_vt_rec_pitch,
					(ParamFloat<px4::params::VT_REC_THR_FW>) _param_vt_rec_thr_fw,
					(ParamFloat<px4::params::VT_REC_THR_MC>) _param_vt_rec_thr_mc,
					(ParamFloat<px4::params::VT_REC_PIT_RT>) _param_vt_rec_pit_rt,
					(ParamFloat<px4::params::VT_REC_VZ>) _param_vt_rec_vz,
					(ParamFloat<px4::params::VT_REC_VXY>) _param_vt_rec_vxy,
					(ParamFloat<px4::params::VT_REC_TMO>) _param_vt_rec_tmo,
					(ParamFloat<px4::params::VT_REC_STICK>) _param_vt_rec_stick
				       )


};
#endif
