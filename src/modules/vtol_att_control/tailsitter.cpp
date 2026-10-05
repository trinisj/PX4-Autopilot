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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Tailsitter::parameters_update()
{
	VtolType::updateParams();

}

/* [2026-10 custom] Recovery state machine, see msg/TailsitterRecovery.msg and the VT_REC_* parameters.
 * Modelled on the manual sequence flown in log_0_2026-10-5-19-15-29: Stabilize pitch-up at minimum thrust, transition at
 * a nose angle of 30-40 deg, Stabilize (attitude only) at low thrust until level, Altitude controller once the climb
 * rate is below 3 m/s, Position controller once the horizontal speed is below 5 m/s. */
void Tailsitter::setRecoveryPhase(uint8_t phase, hrt_abstime now)
{
	if (phase != _rec_phase) {
		_rec_phase = phase;
		_rec_phase_ts = now;
		publishRecovery(now, true);
	}
}

void Tailsitter::publishRecovery(hrt_abstime now, bool force)
{
	if (!force && (now - _rec_pub_ts) < 100_ms) {
		return;
	}

	tailsitter_recovery_s msg{};
	msg.timestamp = now;
	msg.phase = _rec_phase;
	msg.pitch_target = math::radians(_param_vt_rec_pitch.get());
	msg.pitch_rate = math::radians(_param_vt_rec_pit_rt.get());
	msg.thrust_fw = math::constrain(_param_vt_rec_thr_fw.get(), 0.f, 1.f);
	msg.thrust_mc = math::constrain(_param_vt_rec_thr_mc.get(), 0.f, 1.f);
	msg.roll_fw = _rec_roll;
	msg.roll_rate_fw = _rec_roll_rate;
	msg.wings_level = (_rec_level_since != 0);
	_recovery_pub.publish(msg);
	_rec_pub_ts = now;
}

/* Bank angle about the nose in the fixed-wing frame. Body x (MC forward) points DOWN in fixed-wing cruise (belly), body y is the
 * wing axis, so the bank is the angle of the wing axis out of the horizontal plane, measured around the nose:
 * atan2(z-component of body y, z-component of body x), both in NED. 0 = wings level, +/-pi = inverted. The rate is the
 * wrapped difference of that angle, low-pass filtered (independent of the gyro axis conventions). */
void Tailsitter::updateRollLevel(hrt_abstime now)
{
	const Dcmf R{Quatf(_v_att->q)};
	const float bank = atan2f(R(2, 1), R(2, 0));

	if (_rec_roll_ts != 0) {
		const float dt = math::constrain((now - _rec_roll_ts) * 1e-6f, 1e-4f, 0.2f);
		const float rate = matrix::wrap_pi(bank - _rec_roll) / dt;
		const float alpha = dt / (0.2f + dt); // tau 0.2 s
		_rec_roll_rate += alpha * (rate - _rec_roll_rate);
	}

	_rec_roll = bank;
	_rec_roll_ts = now;

	static constexpr float RATE_LIMIT = 0.26f; // [rad/s] 15 deg/s: "no roll rotation in progress"
	const float roll_limit = math::radians(_param_vt_rec_lvl_roll.get());
	const bool within = (roll_limit <= 0.f) || ((fabsf(bank) < roll_limit) && (fabsf(_rec_roll_rate) < RATE_LIMIT));

	if (!within) {
		_rec_level_since = 0;

	} else if (_rec_level_since == 0) {
		_rec_level_since = now;
	}
}

uint32_t Tailsitter::recoveryModeSignature() const
{
	return (_v_control_mode->flag_control_manual_enabled ? 1u : 0u)
	       | (_v_control_mode->flag_control_auto_enabled ? 2u : 0u)
	       | (_v_control_mode->flag_control_position_enabled ? 4u : 0u)
	       | (_v_control_mode->flag_control_altitude_enabled ? 8u : 0u)
	       | (_v_control_mode->flag_control_climb_rate_enabled ? 16u : 0u)
	       | (_v_control_mode->flag_control_attitude_enabled ? 32u : 0u);
}

/* Pilot takeover = the pilot (RC mode switch), the GCS or a failsafe changed the flight mode since the recovery started:
 * vehicle_status.nav_state or the control-mode flags differ. Sticks cancel the recovery only if VT_REC_STICK > 0 (default 0 =
 * off, mode switch only). log_1_2026-10-5-20-03-05: without any release rule the recovery stayed in POS_HOLD until disarm and
 * overrode the setpoints, no stick and no flight-mode switch reached the vehicle for 60 s. */
bool Tailsitter::recoveryPilotTakeover()
{
	if (recoveryModeSignature() != _rec_mode_sig) {
		return true;
	}

	vehicle_status_s status{};

	if (_rec_status_sub.copy(&status) && (status.nav_state != _rec_nav_state)) {
		return true;
	}

	const float stick_limit = _param_vt_rec_stick.get();

	if (stick_limit > FLT_EPSILON) {
		manual_control_setpoint_s manual{};

		if (_rec_manual_sub.copy(&manual) && manual.valid && (hrt_elapsed_time(&manual.timestamp) < 500_ms)) {
			return (fabsf(manual.roll) > stick_limit) || (fabsf(manual.pitch) > stick_limit)
			       || (fabsf(manual.yaw) > stick_limit) || (fabsf(manual.throttle - _rec_thr_ref) > stick_limit);
		}
	}

	return false;
}

void Tailsitter::updateRecovery()
{
	static constexpr float PITCH_REACHED_TOL = 0.052f;     // [rad] 3 deg: manual modes clamp the pitch setpoint to FW_P_LIM_MAX, the vehicle then settles ~1.5 deg above/below it
	static constexpr float LEVEL_TOL = 0.35f;              // [rad] 20 deg: "multicopter level attitude reached"
	static constexpr hrt_abstime ATT_HOLD_MAX = 20_s;      // fallback if the climb rate never gets valid / small

	const hrt_abstime now = hrt_absolute_time();
	updateRollLevel(now);
	const bool wings_level = (_rec_level_since != 0) && ((now - _rec_level_since) > 500_ms); // level for 0.5 s
	const bool enabled = _param_vt_rec_en.get() > 0;
	const bool armed = _v_control_mode->flag_armed;
	const bool fw_requested = _attc->is_fixed_wing_requested();
	const bool quad_chute = _vtol_vehicle_status->fixed_wing_system_failure;

	if (!enabled || !armed || fw_requested || quad_chute) {
		_rec_cancelled = false; // a new command / disarm re-arms the automatic recovery
		_back_trans_from_recovery = false;
		setRecoveryPhase(tailsitter_recovery_s::PHASE_IDLE, now);
		publishRecovery(now, false);
		return;
	}

	if (_rec_cancelled) { // pilot took over earlier: stock behaviour, nothing is overridden
		publishRecovery(now, false);
		return;
	}

	if (_rec_phase != tailsitter_recovery_s::PHASE_IDLE && recoveryPilotTakeover()) {
		_rec_cancelled = true;
		setRecoveryPhase(tailsitter_recovery_s::PHASE_IDLE, now);
		publishRecovery(now, true);
		return;
	}

	// MC-frame pitch theta is -90 deg in fixed-wing cruise; nose above horizon = theta + 90 deg
	const float theta = Eulerf(Quatf(_v_att->q)).theta();
	const float pitch_fw = theta + M_PI_2_F;
	const float elapsed = (now - _rec_phase_ts) * 1e-6f;

	switch (_rec_phase) {
	case tailsitter_recovery_s::PHASE_IDLE:
		if (_vtol_mode == vtol_mode::FW_MODE) { // transition-to-MC command in fixed-wing flight
			_rec_mode_sig = recoveryModeSignature();
			vehicle_status_s status_start{};
			_rec_nav_state = _rec_status_sub.copy(&status_start) ? status_start.nav_state : 0;
			manual_control_setpoint_s manual_start{};
			_rec_thr_ref = (_rec_manual_sub.copy(&manual_start) && manual_start.valid) ? manual_start.throttle : 0.f;
			_rec_level_warned = false;
			/* [2026-10 custom] wings first: with a bank, or a roll in progress, the back transition cannot be flown (the
			 * start attitude has wings level; the MC controller would have to remove the bank as a yaw error). */
			setRecoveryPhase(wings_level ? tailsitter_recovery_s::PHASE_PITCH_UP : tailsitter_recovery_s::PHASE_LEVEL_ROLL, now);
		}

		break;

	case tailsitter_recovery_s::PHASE_LEVEL_ROLL:
		if (_vtol_mode != vtol_mode::FW_MODE) { // something else already left fixed-wing mode
			setRecoveryPhase(tailsitter_recovery_s::PHASE_BACK_TRANSITION, now);

		} else if (wings_level) {
			setRecoveryPhase(tailsitter_recovery_s::PHASE_PITCH_UP, now);

		} else if (elapsed > _param_vt_rec_lvl_tmo.get()) {
			PX4_WARN("recovery: wings not level after %.0f s (bank %.0f deg, rate %.0f deg/s), continuing", (double)elapsed,
				 (double)math::degrees(_rec_roll), (double)math::degrees(_rec_roll_rate));
			_rec_level_warned = true;
			setRecoveryPhase(tailsitter_recovery_s::PHASE_PITCH_UP, now);
		}

		break;

	case tailsitter_recovery_s::PHASE_PITCH_UP: {
			const bool reached = pitch_fw >= math::radians(_param_vt_rec_pitch.get()) - PITCH_REACHED_TOL;
			const bool timeout = elapsed > _param_vt_rec_tmo.get();

			if (_vtol_mode != vtol_mode::FW_MODE) { // something else already left fixed-wing mode
				setRecoveryPhase(tailsitter_recovery_s::PHASE_BACK_TRANSITION, now);

			} else if ((reached || timeout) && (wings_level || elapsed > (_param_vt_rec_tmo.get() + _param_vt_rec_lvl_tmo.get()))) {
				/* the pitch is held at the target (roll setpoint 0) until the wings are level; bounded by VT_REC_LVL_TMO */
				resetTransitionStates();
				_back_trans_from_recovery = true; // start attitude = actual nose pitch, see update_transition_state()
				_vtol_mode = vtol_mode::TRANSITION_BACK;
				setRecoveryPhase(tailsitter_recovery_s::PHASE_BACK_TRANSITION, now);
			}

			break;
		}

	case tailsitter_recovery_s::PHASE_BACK_TRANSITION:
		if (_vtol_mode == vtol_mode::MC_MODE) {
			setRecoveryPhase(tailsitter_recovery_s::PHASE_ATT_HOLD, now);

		} else if (_vtol_mode == vtol_mode::FW_MODE) { // transition was aborted by someone else
			setRecoveryPhase(tailsitter_recovery_s::PHASE_IDLE, now);
		}

		break;

	case tailsitter_recovery_s::PHASE_ATT_HOLD: {
			const bool vz_ok = _local_pos->v_z_valid && PX4_ISFINITE(_local_pos->vz)
					   && (_local_pos->vz > -_param_vt_rec_vz.get()); // NED: climbing = negative
			const bool level = fabsf(theta) < LEVEL_TOL;

			if ((vz_ok && level) || (now - _rec_phase_ts) > ATT_HOLD_MAX) {
				setRecoveryPhase(tailsitter_recovery_s::PHASE_ALT_HOLD, now);
			}

			break;
		}

	case tailsitter_recovery_s::PHASE_ALT_HOLD: {
			const bool vxy_ok = _local_pos->v_xy_valid && PX4_ISFINITE(_local_pos->vx) && PX4_ISFINITE(_local_pos->vy)
					    && (Vector2f(_local_pos->vx, _local_pos->vy).norm() < _param_vt_rec_vxy.get());

			if (vxy_ok) {
				setRecoveryPhase(tailsitter_recovery_s::PHASE_POS_HOLD, now);
			}

			break;
		}

	case tailsitter_recovery_s::PHASE_POS_HOLD: // stays until the pilot takes over, disarm or a new transition to fixed-wing
	default:
		break;
	}

	publishRecovery(now, false);
}

void Tailsitter::update_vtol_state()
{
	updateRecovery(); // [2026-10 custom] may start the pitch-up phase or the back transition itself

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/


	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		/* [2026-09 custom] Stock: switch to MC mode immediately, from any
		 * state. From FW_MODE at speed that hands a fast, nose-level
		 * vehicle straight to the hover controller, which commands a
		 * near-vertical attitude at once. log_0_2026-9-27-16-18-08.ulg:
		 * quad-chute at 50 m/s, MC setpoint jumped to 72 deg nose-up and
		 * attitude control was lost.
		 *
		 * From FW_MODE, go through the normal ramped back transition
		 * (VT_B_TRANS_DUR, pitch-threshold completion) - the same path a
		 * pilot-commanded back transition takes. Other states keep the
		 * stock behaviour (front transition is still slow, so an instant
		 * MC switch there is the safer choice). */
		switch (_vtol_mode) {
		case vtol_mode::FW_MODE:
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_BACK: {
				const float pitch = Eulerf(Quatf(_v_att->q)).theta();

				if (pitch >= PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC || _time_since_trans_start > _param_vt_b_trans_dur.get()) {
					_vtol_mode = vtol_mode::MC_MODE;
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_P1:
			/* [2026-09 custom] log_6_2026-9-27-18-52-50.ulg: transition timeout
			 * quad-chute at 86 m/s with the nose 3.7 deg above the horizon. The
			 * instant MC switch stepped the setpoint to 90 deg nose-up and the
			 * vehicle oscillated +-20 deg for ~5 s until it had slowed below
			 * 50 m/s. Once the front transition is fast, treat it like FW. */
			if (frontTransitionIsFast()) {
				abortFrontTransitionToBack();

			} else {
				_transition_start_timestamp = hrt_absolute_time();
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;

		default:
			if (_vtol_mode != vtol_mode::MC_MODE) {
				_transition_start_timestamp = hrt_absolute_time();
			}

			_vtol_mode = vtol_mode::MC_MODE;
			break;
		}

	} else if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_mode) { // user switchig to MC mode
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			/* [2026-10 custom] recovery: stay in fixed-wing mode while the pitch-up phase runs; updateRecovery() starts the
			 * back transition when the pitch is reached */
			if (_rec_phase == tailsitter_recovery_s::PHASE_PITCH_UP || _rec_phase == tailsitter_recovery_s::PHASE_LEVEL_ROLL) {
				break;
			}

			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// [2026-09 custom] stock: failsafe into multicopter mode immediately.
			// Keep that while still slow; once fast, use the ramped back transition.
			if (frontTransitionIsFast()) {
				abortFrontTransitionToBack();

			} else {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			const float pitch = Eulerf(Quatf(_v_att->q)).theta();

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC || _time_since_trans_start > _param_vt_b_trans_dur.get()) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			resetTransitionStates();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {

				if (isFrontTransitionCompleted()) {
					_vtol_mode = vtol_mode::FW_MODE;
					_trans_finished_ts = hrt_absolute_time();
				}

				break;
			}

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_mode = vtol_mode::FW_MODE;
			_trans_finished_ts = hrt_absolute_time();
			break;
		}
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// we need the incoming (virtual) mc attitude setpoints to be recent, otherwise return (means the previous setpoint stays active)
	if (_mc_virtual_att_sp->timestamp < (now - 1_s)) {
		return;
	}

	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			_trans_rot_axis = z.cross(Vector3f(0.f, 0.f, -1.f));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			const float yaw_sp = atan2f(z(1), z(0));

			/* [2026-10 custom] Attitude first. Stock: the start attitude is the current FW pitch SETPOINT. That setpoint is
			 * whatever the FW controller (TECS) asks for at that instant - which is not the attitude the vehicle is flying.
			 * log_4_2026-10-5-20-36-48 / log_1_2026-10-5-20-38-39: nose +32 deg, TECS height-rate setpoint -5 m/s (the vehicle
			 * was climbing) -> pitch setpoint -30 deg (FW_P_LIM_MIN) -> 62 deg step in 0.25 s, nose-down dive to 125 m/s,
			 * quad-chute at the minimum altitude. A back transition now always starts from the ACTUAL nose pitch (above
			 * horizon, wings level, heading = nose heading), so the setpoint is continuous whatever the FW controller was
			 * doing; altitude is allowed to change while the attitude is brought to the hover attitude. z is the nose
			 * direction (NED). */
			const float pitch_body = asinf(math::constrain(-z(2), -1.f, 1.f));
			_q_trans_start = Eulerf(0.f, pitch_body, yaw_sp);
			_back_trans_from_front = false;
			_back_trans_from_recovery = false;

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			const Eulerf setpoint_euler(Quatf(_mc_virtual_att_sp->q_d));
			_q_trans_start = Eulerf(0.f, setpoint_euler.theta(), setpoint_euler.psi());
			// [2026-09 custom] forward tilt already present at the start (MC frame: forward = negative theta)
			_trans_start_tilt = -setpoint_euler.theta();
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1.f, 0.f, 0.f);
			_trans_rot_axis = -x.cross(Vector3f(0.f, 0.f, -1.f));
		}

		_q_trans_sp = _q_trans_start;
		_trans_progress_time = 0.f; // [2026-09 custom] fresh gated timer for this transition
		_lift_frac_filt = 0.f;      // [2026-09 custom] fresh lift estimate for this transition
		_back_trans_progress_time = 0.f; // [2026-09 custom] fresh speed-gated timer, see VT_B_TR_SPD_GATE
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	const float cos_tilt = math::constrain(_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) -
					       _q_trans_sp(2) * _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3), -1.f, 1.f);
	const float tilt = acosf(cos_tilt);

	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {

		/* [2026-09 custom] The stock schedule advances purely with
		 * _time_since_trans_start, reaching close to 90 deg on a fixed
		 * clock regardless of whether the airframe has picked up enough
		 * airspeed/lift to be tilted that far yet. For an airframe that
		 * needs high speed before it generates meaningful lift, that can
		 * commit to a tilt with almost no vertical thrust component left
		 * and nothing yet aerodynamic to replace it - an uncontrolled
		 * sink, not a transition.
		 *
		 * _trans_progress_time replaces _time_since_trans_start as the
		 * clock fed into the tilt-rate calculation below. It only
		 * advances while the measured sink rate is under
		 * VT_F_TR_SINK_MAX; otherwise it holds, which holds the tilt
		 * setpoint at its current value rather than driving it further.
		 * VT_TRANS_MIN_TM/VT_TRANS_TIMEOUT are untouched - they still key
		 * off the real _time_since_trans_start, so the overall transition
		 * time budget (and the automatic abort if it runs out) is exactly
		 * as before; only how fast tilt is allowed to advance inside that
		 * budget responds to real climb performance now.
		 *
		 * _local_pos->vz is NED, positive = descending. If position data
		 * is momentarily unavailable, fail safe by not advancing (treat
		 * as if sinking) rather than assuming a good climb rate. */
		const bool have_local_pos = (_local_pos != nullptr) && PX4_ISFINITE(_local_pos->vz) && _local_pos->v_z_valid;
		const float sink_rate = have_local_pos ? _local_pos->vz : _param_vt_f_tr_sink_max.get() + 1.f;

		updateLiftEstimate(); // [2026-09 custom] used below and by isFrontTransitionCompletedBase()

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_f_trans_dur.get(), 0.1f);

		/* [2026-09 custom] Lift-paced tilt cap.
		 *
		 * log_0_2026-9-27-16-18-08.ulg: the stock clock tilted the nose
		 * 18 deg/s while speed lagged far behind - 30 deg of tilt at only
		 * 4.9 m/s, 58 deg at 18.7 m/s with the fins carrying 35% of the
		 * weight. The vehicle was climbing the whole time, so the sink
		 * gate alone never engaged.
		 *
		 * Vertical balance at tilt theta with usable thrust k*W:
		 *     k * W * cos(theta) + L = W
		 * so the largest tilt that can still hold altitude is
		 *     theta_cap = acos((1 - L/W) / k)
		 * with L/W the measured lift fraction and
		 *     k = (MPC_THR_MAX - MPC_THR_XY_MARG) / hover_thrust
		 * (keeps the same attitude-control headroom as the MC controller).
		 * For this airframe (hover ~0.5, max 1.0, margin 0.3 -> k ~1.4):
		 *     L/W = 0    -> 44 deg      L/W = 0.35 -> 62 deg
		 *     L/W = 0.7  -> 78 deg      L/W = 1.0  -> 90 deg
		 * i.e. the nose only goes down as far as the fins have shown they
		 * can make up for - pitching over as lift actually builds, the way a
		 * pilot flies it by feel, but driven by the measurement.
		 *
		 * Tilt still advances at most at the VT_F_TRANS_DUR rate and only
		 * while the sink gate allows. If lift falls and the setpoint is above
		 * the cap, the nose is eased back up at the same rate instead of
		 * holding a tilt that cannot be sustained. */
		const float hover = hoverThrust();
		const float k_usable = math::max((_param_mpc_thr_max.get() - _param_mpc_thr_xy_marg.get()) / hover, 1.05f);
		_tilt_cap = acosf(math::constrain((1.f - _lift_frac_filt) / k_usable, 0.f, 1.f));

		/* The setpoint is _q_trans_start rotated by progress * rate, so the
		 * absolute forward tilt is _trans_start_tilt + progress * rate.
		 * [fix] The first version compared progress * rate alone against the
		 * cap, so a transition started while already tilted forward ran
		 * ahead of the cap by the starting tilt. log_7_2026-9-27-19-07-40.ulg:
		 * 2nd transition started at 8.5 deg (still settling after a back
		 * transition), the nose went ~8 deg lower than the cap allowed, the
		 * vehicle started sinking, the MC altitude loop drove collective to
		 * 1.0 and motors 0/2 saturated, so the nose could not be raised
		 * again. The 1st transition started from 0.2 deg and was fine.
		 * Easing back may rotate past the start attitude, but never beyond
		 * vertical. */
		const float progress_cap = (_tilt_cap - _trans_start_tilt) / trans_pitch_rate;
		const float progress_min = -math::max(_trans_start_tilt, 0.f) / trans_pitch_rate;

		if (_trans_progress_time > progress_cap) {
			_trans_progress_time = math::max(progress_cap, _trans_progress_time - _transition_dt);

		} else if (sink_rate < _param_vt_f_tr_sink_max.get()) {
			_trans_progress_time = math::min(progress_cap, _trans_progress_time + _transition_dt);
		}

		_trans_progress_time = math::max(_trans_progress_time, progress_min);

		if (tilt < M_PI_2_F - math::radians(_param_fw_psp_off.get())) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _trans_progress_time * trans_pitch_rate)) * _q_trans_start;
		}

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_b_trans_dur.get(), 0.1f);

		/* [2026-09 custom] HANDOFF 9 / VT_B_TR_SPD_GATE: stock advances this
		 * purely on _time_since_trans_start, regardless of speed.
		 * log_3_2026-9-27-22-54-28.ulg: back transition started at 92 m/s,
		 * commanded nose 7 -> 52 deg while actual nose went -14 deg and
		 * falling (fins overpowering the differential-thrust pitch
		 * authority), altitude oscillated, roll excursions up to ~180 deg.
		 * Hold the schedule at its start attitude until speed drops under
		 * VT_B_TR_SPD_GATE - decelerate on drag first, then raise the nose
		 * for real - instead of fighting a nose-up the airframe cannot win
		 * yet. VT_B_TRANS_DUR/VT_TRANS_TIMEOUT still bound the total time.
		 * Missing speed data fails safe (treated as still fast - hold). */
		const bool have_speed = (_local_pos != nullptr) && _local_pos->v_xy_valid
					&& PX4_ISFINITE(_local_pos->vx) && PX4_ISFINITE(_local_pos->vy);
		const float speed = have_speed ? Vector2f(_local_pos->vx, _local_pos->vy).norm()
				    : _param_vt_b_tr_spd_gate.get() + 1.f;

		if (speed < _param_vt_b_tr_spd_gate.get()) {
			_back_trans_progress_time = math::min(_back_trans_progress_time + _transition_dt,
							       _param_vt_b_trans_dur.get());
		}

		if (tilt > 0.01f) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _back_trans_progress_time * trans_pitch_rate)) * _q_trans_start;
		}
	}

	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
		const float progress = math::constrain(_time_since_trans_start / B_TRANS_THRUST_BLENDING_DURATION, 0.f, 1.f);
		blendThrottleBeginningBackTransition(progress);
	}

	_v_att_sp->timestamp = hrt_absolute_time();

	const Eulerf euler_sp(_q_trans_sp);
	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[0] = -_last_thr_in_mc;
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// Motors
	if (_vtol_mode == vtol_mode::FW_MODE) {

		_thrust_setpoint_0->xyz[2] = -_vehicle_thrust_setpoint_virtual_fw->xyz[0];

		/* allow differential thrust if enabled */
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] * _param_vt_fw_difthr_s_y.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::PITCH_BIT)) {
			_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] * _param_vt_fw_difthr_s_p.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::ROLL_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_r.get();
		}

		// for the short period after switching to FW where there is no thrust published yet from the FW controller,
		// keep publishing the last MC thrust to keep the motors running
		if (hrt_elapsed_time(&_trans_finished_ts) < 50_ms) {
			_thrust_setpoint_0->xyz[2] = _last_thr_in_mc;
			_torque_setpoint_0->xyz[0] = 0.f;
			_torque_setpoint_0->xyz[1] = 0.f;
			_torque_setpoint_0->xyz[2] = 0.f;
		}

	} else {
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// for the short period after starting the backtransition where there is no thrust published yet from the MC controller,
		// keep publishing the last FW thrust to keep the motors running
		if (_vtol_mode != vtol_mode::TRANSITION_FRONT_P1 && hrt_elapsed_time(&_transition_start_timestamp) < 50_ms) {
			_thrust_setpoint_0->xyz[2] = -_last_thr_in_fw_mode;
		}

		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
	}

	// Control surfaces
	if (!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE) {
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
	}
}


/* [2026-09 custom] Measured lift fraction during the front transition.
 *
 * Vertical force balance, normalised by weight:
 *   a_up/g = T_up/W + L_up/W - 1
 *   T_up/W = (thrust / hover_thrust) * cos(tilt)
 * =>  L_up/W = 1 + a_up/g - (thrust / hover_thrust) * cos(tilt)
 *
 * a_up = -vehicle_local_position.az (NED). thrust is the normalised MC
 * collective actually commanded during the transition. hover_thrust comes
 * from hover_thrust_estimate when valid, else MPC_THR_HOVER.
 *
 * Validated offline on log_13_2026-9-27-15-57-39.ulg: ~0.01 in hover
 * (correctly ~0), 0.07 at 9 m/s, 0.17 at 16 m/s, 0.40 at 21.5 m/s.
 * Assumes thrust is roughly linear in the normalised command
 * (THR_MDL_FAC=0 in SITL). With a nonzero THR_MDL_FAC on the real vehicle
 * the absolute value is approximate, so re-check VT_F_TR_LIFT_MIN against
 * real logs before relying on it outdoors. */
float Tailsitter::hoverThrust()
{
	hover_thrust_estimate_s hte;

	if (_hover_thrust_estimate_sub.update(&hte) && hte.valid && PX4_ISFINITE(hte.hover_thrust)) {
		_hover_thrust = hte.hover_thrust;
	}

	const float hover = (PX4_ISFINITE(_hover_thrust) && _hover_thrust > 0.05f)
			    ? _hover_thrust : _param_mpc_thr_hover.get();

	return math::max(hover, 0.05f);
}

void Tailsitter::updateLiftEstimate()
{
	static constexpr float GRAVITY = 9.80665f;

	const float hover = hoverThrust();

	const float thrust = -_mc_virtual_att_sp->thrust_body[2];

	if ((_local_pos == nullptr) || !PX4_ISFINITE(_local_pos->az) || !PX4_ISFINITE(thrust) || (hover < 0.05f)) {
		return; // keep the last estimate; never invent lift from missing data
	}

	const float cos_tilt = Dcmf(Quatf(_v_att->q))(2, 2);
	const float a_up = -_local_pos->az;
	const float lift_frac = 1.f + a_up / GRAVITY - (thrust / hover) * cos_tilt;

	const float alpha = _transition_dt / (LIFT_EST_TAU + _transition_dt);
	_lift_frac_filt += alpha * (math::constrain(lift_frac, -1.f, 2.f) - _lift_frac_filt);
}

/* [2026-09 custom] A front transition counts as "fast" once the nose is past
 * the same pitch threshold used for completing it (within 30 deg of the
 * horizon). From there an instant MC switch steps the setpoint by 60+ deg
 * while the fins hold the nose to the flight path; below it, the stock
 * instant MC switch is kept. */
bool Tailsitter::frontTransitionIsFast()
{
	const float pitch = Eulerf(Quatf(_v_att->q)).theta();
	return pitch <= PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW;
}

void Tailsitter::abortFrontTransitionToBack()
{
	resetTransitionStates();

	/* The stock back transition holds, then blends from, the last FW-mode
	 * throttle (fill_actuator_outputs, blendThrottleBeginningBackTransition).
	 * Coming from a front transition that value is stale or zero, which would
	 * cut thrust for up to 0.5 s. Seed it with the thrust actually applied now
	 * (MC collective, positive magnitude) so thrust stays continuous. */
	_last_thr_in_fw_mode = -_mc_virtual_att_sp->thrust_body[2];

	_back_trans_from_front = true;
	_flag_was_in_trans_mode = false; // re-run the transition initialisation for the back transition
	_vtol_mode = vtol_mode::TRANSITION_BACK;
}

bool Tailsitter::isFrontTransitionCompletedBase()
{
	/* [2026-09 custom] Stock behaviour: with pitch past the threshold,
	 * complete on airspeed >= VT_ARSP_TRANS, or - if airspeed is not
	 * available - complete on pitch alone. That second branch is what
	 * handed control to the FW controller at 23 m/s with the fins carrying
	 * only ~40% of the weight in log_13_2026-9-27-15-57-39.ulg; TECS then
	 * cut throttle to ~0 and the vehicle dived.
	 *
	 * Now all three must hold:
	 *  1. pitch past PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW (unchanged)
	 *  2. speed >= VT_ARSP_TRANS, using calibrated airspeed if available,
	 *     otherwise horizontal groundspeed. If neither is available the
	 *     transition does not complete; VT_TRANS_TIMEOUT then quad-chutes
	 *     back to MC, which is the safe outcome.
	 *  3. measured lift fraction >= VT_F_TR_LIFT_MIN (0 disables), i.e.
	 *     the fins are demonstrably carrying the weight before the FW
	 *     controller is trusted with the vehicle. */
	const float pitch = Eulerf(Quatf(_v_att->q)).theta();

	if (pitch > PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW) {
		return false;
	}

	float speed = NAN;
	const char *speed_src = "none";

	if (PX4_ISFINITE(_attc->get_calibrated_airspeed())) {
		speed = _attc->get_calibrated_airspeed();
		speed_src = "airspeed";

	} else if ((_local_pos != nullptr) && _local_pos->v_xy_valid
		   && PX4_ISFINITE(_local_pos->vx) && PX4_ISFINITE(_local_pos->vy)) {
		speed = Vector2f(_local_pos->vx, _local_pos->vy).norm();
		speed_src = "groundspeed";
	}

	const bool speed_ok = PX4_ISFINITE(speed) && (speed >= _param_vt_arsp_trans.get());

	const float lift_min = _param_vt_f_tr_lift_min.get();
	const bool lift_ok = (lift_min <= 0.f) || (_lift_frac_filt >= lift_min);

	/* [2026-09 custom] Do not hand a sinking vehicle to the FW controller.
	 * log_7_2026-9-27-19-07-40.ulg: 1st handover at 0.17 m/s sink -> 60 s of
	 * FW flight; 2nd handover at 2.65 m/s sink -> quad-chute 1.85 s later.
	 * Threshold: half of VT_F_TR_SINK_MAX (1.5 m/s at the default 3.0). */
	const bool not_sinking = (_local_pos != nullptr) && _local_pos->v_z_valid && PX4_ISFINITE(_local_pos->vz)
				 && (_local_pos->vz < 0.5f * _param_vt_f_tr_sink_max.get());

	if (speed_ok && lift_ok && not_sinking) {
		/* nose above horizon at handover = the trim pitch this airframe
		 * actually needed to hold altitude at this speed. Use it to set
		 * FW_PSP_OFF so the FW controller does not start from level. */
		PX4_INFO("front transition complete: %s %.1f m/s, lift/W %.2f, trim pitch %.1f deg, sink %.2f m/s",
			 speed_src, (double)speed, (double)_lift_frac_filt,
			 (double)math::degrees(M_PI_2_F + pitch), (double)_local_pos->vz);
		return true;
	}

	return false;
}

void Tailsitter::blendThrottleAfterFrontTransition(float scale)
{
	// note: MC throttle is negative (as in negative z), while FW throttle is positive (positive x)
	_v_att_sp->thrust_body[0] = scale * _v_att_sp->thrust_body[0] + (1.f - scale) * (-_last_thr_in_mc);
}

void Tailsitter::blendThrottleBeginningBackTransition(float scale)
{
	_v_att_sp->thrust_body[2] = scale * _v_att_sp->thrust_body[2] + (1.f - scale) * (-_last_thr_in_fw_mode);
}
