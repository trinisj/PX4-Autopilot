/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	_vel_xy_controlled = PX4_ISFINITE(setpoint.velocity[0]) || PX4_ISFINITE(setpoint.position[0]);		// 261003

}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

// 261003
/*
	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}
*/

	// [2026-09 trinidrone] collective held up by the aero thrust floor: do not wind down
	if (_aero_floor_active && vel_error(2) >= 0.f) {
		vel_error(2) = 0.f;
	}

	// [2026-09 trinidrone] zoom brake ignores the altitude loop on purpose: do not wind it up
	if (_zoom_brake_active) {
		vel_error(2) = 0.f;
	}

	// [2026-09 trinidrone] speed mode could not follow the vertical need: freeze that direction
	if (_speed_mode_weight > 0.f) {
		if (_spd_tilt_capped && vel_error(2) >= 0.f) {  // wants down, nose already at the tilt limit
			vel_error(2) = 0.f;
		}

		if (_spd_thr_saturated && vel_error(2) <= 0.f) { // wants up, thrust already at max
			vel_error(2) = 0.f;
		}
	}

	if (_aero_limit_active) {
		/* [2026-09 trinidrone] The aero angle limit chose this thrust direction
		 * to keep the fins' moment inside the motors' authority. The stock
		 * saturation below would shorten only the horizontal part, i.e. rotate
		 * the thrust (= nose) back up, straight into the angle the limit just
		 * removed. Here attitude has priority over altitude: keep the direction,
		 * scale the length, and stop the vertical integrator from winding up
		 * for upward thrust it cannot get. */
		const float thr_norm = _thr_sp.norm();

		if (thr_norm > _lim_thr_max) {
			_thr_sp *= _lim_thr_max / thr_norm;

			if (vel_error(2) <= 0.f) { // NED: negative = wants more upward thrust
				vel_error(2) = 0.f;
			}
		}

	} else {
		// Prioritize vertical control while keeping a horizontal margin
		const Vector2f thrust_sp_xy(_thr_sp);
		const float thrust_sp_xy_norm = thrust_sp_xy.norm();
		const float thrust_max_squared = math::sq(_lim_thr_max);

		// Determine how much vertical thrust is left keeping horizontal margin
		const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
		const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

		// Saturate maximal vertical thrust
		_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

		// Determine how much horizontal thrust is left after prioritizing vertical control
		const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
		float thrust_max_xy = 0.f;

		if (thrust_max_xy_squared > 0.f) {
			thrust_max_xy = sqrtf(thrust_max_xy_squared);
		}

		// Saturate thrust in horizontal direction
		if (thrust_sp_xy_norm > thrust_max_xy) {
			_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
		}
	}

	/* [2026-09 trinidrone] In speed mode (or zoom brake) the horizontal demand no
	 * longer maps to horizontal thrust (thrust comes from the along-track part,
	 * tilt from the vertical need), so the stock tracking ARW below compares
	 * unrelated quantities. Instead stop the along-track integrator where its
	 * output has no effect: thrust already at max (s >= 1) and wanting more,
	 * thrust at the floor (s <= 0) and wanting less, or in the zoom brake,
	 * which ignores the demand magnitude while the manual velocity setpoint
	 * ramps down far faster than the vehicle can slow. */
	const bool speed_mode_xy = (_speed_mode_weight > 0.f) && Vector2f(vel_error).isAllFinite();

	if (speed_mode_xy) {
		const float e_long = Vector2f(vel_error).dot(_spd_vel_dir);

		if (_zoom_brake_active || (_spd_s_sat_hi && e_long > 0.f) || (_spd_s_sat_lo && e_long < 0.f)) {
			vel_error.xy() = Vector2f(vel_error) - _spd_vel_dir * e_long;
		}
	}

/******************************************************************************/	

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.

	if (!speed_mode_xy && (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared())) {	// 261003
//	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		const float arw_gain = 2.f / _gain_vel_p(0);
		const Vector2f acc_sp_xy = _acc_sp.xy();

		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	_aero_limit_active = _limitAeroAngle(body_z);		// 261003
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));

// 261003
//	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);

	/* [2026-09 trinidrone] Collective floor at speed (MPC_TS_THR_FLR), independent
	 * of the angle limit. log_12_2026-9-27-23-37-37.ulg: at a constant 64 deg
	 * stick tilt the vehicle held at 42.5 m/s with collective 0.44; as it kept
	 * accelerating, lift grew, the altitude loop cut the collective 0.44 -> 0.19,
	 * one motor reached 0 and the nose could no longer be held against the fins.
	 * Keeping the collective up keeps differential-thrust range; the surplus
	 * shows up as a climb (attitude before altitude). Only above
	 * AERO_FLOOR_MIN_SPEED so hover, descent and landing stay stock. */
	_aero_floor_active = false;
	const bool fast_enough = _vel.isAllFinite() && (Vector2f(_vel).norm() >= AERO_FLOOR_MIN_SPEED);

	if (fast_enough && (_aero_thr_floor > 0.f) && (collective_thrust > -_aero_thr_floor)) {
		collective_thrust = -_aero_thr_floor;
		_aero_floor_active = true;
	}

	// [2026-09 trinidrone] speed mode: thrust from the horizontal demand, tilt from the vertical need
	_speedModeBlend(body_z, collective_thrust, thrust_ned_z);
/*********************************************************************************************/

	_thr_sp = body_z * collective_thrust;
}

// 261003
/* [2026-09 trinidrone] Speed mode for a tailsitter flown in MC mode at speed
 * (MPC_TS_SPD_ON > 0).
 *
 * Stock MC: the horizontal demand sets the TILT, the altitude loop sets the
 * COLLECTIVE. At speed that is the wrong way round for this airframe.
 * log_0_2026-9-28-00-08-21.ulg, Altitude mode, full forward stick:
 *   - the stick saturates the horizontal acceleration at 3 g (29.3 m/s^2),
 *     i.e. tilt 71.6 deg, whatever MPC_MAN_TILT_MAX says;
 *   - lift carried 0.75-0.85 W, so the altitude loop wanted less than the
 *     MPC_TS_THR_FLR floor; collective sat at exactly 0.40 the whole time;
 *   - fixed tilt x fixed thrust = fixed forward force: speed plateaued at
 *     41-43 m/s while the surplus went into a 3-4 m/s climb (78 -> 300 m)
 *     against a max-rate descent setpoint.
 * Nothing in the loop turns the pilot's "go faster" into thrust.
 *
 * Here, above MPC_TS_SPD_ON (blended in over SPEED_MODE_BLEND m/s):
 *   thrust  T = floor + (max - floor) * s,  s = |a_xy| / SPEED_MODE_ACC_FULL
 *           (stick in Altitude mode, velocity loop output in Position mode)
 *   tilt      = acos(vertical need / T)  -> the nose angle that holds the
 *           altitude the loop asks for with that thrust; limited to
 *           [SPEED_MODE_TILT_MIN, MPC_TILTMAX_AIR]
 *   direction = horizontal demand.
 * As speed and lift grow the vertical need drops and the nose comes down on
 * its own (smaller angle of attack, smaller fin moment) - pitch follows speed,
 * throttle sets speed.
 *
 * Braking (demand small or pointing backwards):
 *   - below MPC_TS_BRK_SPD: stock, i.e. the flare that works (38 -> 17 m/s in
 *     2.5 s, log_12 116 s);
 *   - above it: zoom brake, see _zoomBrake(). A stock flare from 58.5 m/s
 *     could not raise the nose (log_0_2026-9-28 44.5 s: pitch torque
 *     saturated, motors 0.80/0.00/0.80/0.00, tilt stuck 55-68 deg against
 *     0.7) and turned into a zoom climb and a roll departure. */
void PositionControl::_speedModeBlend(Vector3f &body_z, float &collective_thrust, const float thrust_ned_z)
{
	_spd_tilt_capped = false;
	_spd_thr_saturated = false;
	_spd_s_sat_hi = false;
	_spd_s_sat_lo = false;
	_speed_mode_weight = 0.f;
	_zoom_brake_active = false;

	if (!(_spd_mode_on > 0.f) || !_vel.isAllFinite() || !_acc_sp.isAllFinite()) {
		_spd_braking = false;
		return;
	}

	const Vector2f vel_xy(_vel);
	const float speed_xy = vel_xy.norm();
	const float weight = math::constrain((speed_xy - _spd_mode_on) / SPEED_MODE_BLEND, 0.f, 1.f);

	if (weight <= 0.f) {
		_spd_braking = false;
		return;
	}

	/* [2026-09 trinidrone] Split the demand along / across the track instead
	 * of using its direction and magnitude. The first version took the nose
	 * direction from the acceleration vector and switched speed mode <-> stock
	 * (or zoom brake) on "demand forward and > 1 m/s^2". Fine with a steady
	 * stick in Altitude mode; in Position mode the demand is the velocity PID
	 * output, which sits near zero in steady flight and crosses it all the
	 * time - every crossing stepped the attitude setpoint between ~80 deg
	 * (speed mode) and the stock/zoom attitude, and lateral corrections swung
	 * the nose heading. Now:
	 *   along-track demand  -> thrust only (continuous)
	 *   cross-track demand  -> small, continuous nose offset from the track
	 *   braking             -> entered/left with hysteresis, see below */
	_spd_vel_dir = vel_xy / speed_xy;
	const Vector2f acc_xy(_acc_sp);
	const float a_long = acc_xy.dot(_spd_vel_dir);
	const Vector2f a_lat = acc_xy - _spd_vel_dir * a_long;

	/* Braking hysteresis. Altitude mode (no velocity setpoint): a centred stick
	 * (a_long ~ 0) means "stop", as before. Position mode: the velocity loop
	 * asks for braking with a clearly negative along-track demand; small
	 * negative values in steady flight are just the PID working and must not
	 * trigger it. */
	const float enter = _vel_xy_controlled ? -SPEED_MODE_BRK_ENTER_POS : SPEED_MODE_ACC_MIN;
	const float leave = _vel_xy_controlled ? -SPEED_MODE_BRK_LEAVE_POS : 3.f * SPEED_MODE_ACC_MIN;

	if (_spd_braking) {
		_spd_braking = (a_long < leave);

	} else {
		_spd_braking = (a_long < enter);
	}

	if (_spd_braking) {
		if (speed_xy >= _aero_brake_speed) {
			_zoomBrake(body_z, collective_thrust, weight);
		}

		return; // below MPC_TS_BRK_SPD: stock (flare / normal MC braking)
	}

	const float s_raw = a_long / SPEED_MODE_ACC_FULL;
	const float s = math::constrain(s_raw, 0.f, 1.f);
	_spd_s_sat_hi = (s_raw >= 1.f);
	_spd_s_sat_lo = (s_raw <= 0.f);

	// continuous: 3 g across the track tilts the nose heading 45 deg off the track
	const Vector2f dir = Vector2f(_spd_vel_dir + a_lat / SPEED_MODE_ACC_FULL).normalized();

	const float thr_min = math::max(_aero_thr_floor, _lim_thr_min);
	float thrust = thr_min + (_lim_thr_max - thr_min) * s;

	const float need = -thrust_ned_z; // upward thrust the altitude loop wants [norm], may be <= 0
	const float cos_tilt_max = cosf(_lim_tilt);
	const float cos_tilt_min = cosf(SPEED_MODE_TILT_MIN);

	// not enough thrust to hold altitude even at the steepest allowed nose: raise it
	if (need > thrust * cos_tilt_min) {
		thrust = need / cos_tilt_min;
	}

	if (thrust > _lim_thr_max) {
		thrust = _lim_thr_max;
		_spd_thr_saturated = true;
	}

	float cos_tilt = need / thrust;

	if (cos_tilt <= cos_tilt_max) {
		cos_tilt = cos_tilt_max;
		_spd_tilt_capped = true; // lift alone exceeds what is needed: cannot shed more vertical thrust
	}

	cos_tilt = math::min(cos_tilt, cos_tilt_min);
	const float sin_tilt = sqrtf(math::max(1.f - cos_tilt * cos_tilt, 0.f));

	// body_z points opposite to the thrust (= nose); NED
	const Vector3f body_z_speed(-dir(0) * sin_tilt, -dir(1) * sin_tilt, cos_tilt);

	// scalar * Vector3f yields a plain Matrix<float, 3, 1> (no normalized()), so rebuild a Vector3f
	body_z = Vector3f(body_z * (1.f - weight) + body_z_speed * weight).normalized();
	collective_thrust = (1.f - weight) * collective_thrust + weight * (-thrust);
	_speed_mode_weight = weight;
}

/* [2026-09 trinidrone] Aerodynamic angle limit for a tailsitter flown in MC mode.
 *
 * In MC mode the thrust axis is the nose. The X fins pull the nose towards the
 * velocity vector with a moment of roughly  q * angle,  q = rho/2 * V^2, and
 * the only thing holding it off is differential thrust - whose range shrinks as
 * lift grows and the altitude loop lowers the collective (motors floor at 0).
 *
 * log_12_2026-9-27-23-37-37.ulg, Altitude mode, stick held at ~64 deg tilt
 * (nose ~26 deg above horizon, level flight = ~25 deg nose-to-velocity angle):
 *   95 s, 42.5 m/s, angle ~28 deg (incl. 11 deg sideslip), collective 0.44,
 *         one motor down to 0.19 - holding.
 *   97.2 s, 44.2 m/s: collective 0.38 -> 0.19, motor 1 at 0.00, tilt creeps
 *         64 -> 74 -> 90 deg against a constant 64 deg setpoint, then a
 *         tumble / limit cycle (tilt 46 <-> 100 deg, thrust 1.0 <-> 0.1)
 *         until the stick is released.
 *   132 s, 53 m/s, same thing with yaw stick in (sideslip ~20 deg).
 * The last working point of the front transition (91 m/s, 4.7 deg) and the
 * onset above give angle*V^2 ~ 40e3 (works) vs ~45..50e3 (fails) deg*(m/s)^2.
 *
 * So: limit the total angle between nose and velocity (angle of attack and
 * sideslip together - X fins act in both planes) to  k / V^2.  Above that the
 * nose is rotated towards the velocity vector, in the plane of the two, until
 * the angle equals the limit. At low speed the limit exceeds 90 deg and has no
 * effect; it tightens smoothly as speed rises (k = 30e3: 25 m/s -> 48 deg,
 * 40 m/s -> 19 deg, 60 m/s -> 8 deg, 90 m/s -> 3.7 deg).
 *
 * Because it is relative to the velocity vector and not to the horizon, a
 * pull-up is still possible: the nose may sit up to the limit above the flight
 * path, lift bends the path up, the nose follows - a zoom climb that bleeds
 * speed, instead of an attempt to pitch the nose up against the fins.
 *
 * Velocity is the ground-relative estimate (no airspeed sensor); in wind the
 * true angle differs by the wind component.
 *
 * CAVEAT - EXPERIMENTAL, keep MPC_TS_AOA_K = 0 for now. In MC mode the
 * collective is set by the vertical need (thrust_ned_z / cos(tilt)), so a
 * larger forced tilt also means much more forward thrust. Replayed on the log
 * above (95 s): tilt 64 -> 75 deg, collective 0.43 -> 0.73, forward thrust
 * 0.39 -> 0.70. Above ~35 m/s that accelerates the vehicle towards the
 * high-speed trim (~85-90 m/s) whatever the stick/velocity setpoint says, and
 * at the top end the limit (3.7 deg at 90 m/s with k = 30e3) is below the
 * trim angle seen there (4.7 deg), leaving no margin to slow down. It needs a
 * speed-management counterpart before it is flown. */
bool PositionControl::_limitAeroAngle(Vector3f &body_z) const
{
	if (!(_aero_angle_k > 0.f) || !_vel.isAllFinite()) {
		return false;
	}

	const float speed = _vel.norm();

	if (speed < AERO_LIMIT_MIN_SPEED) {
		return false;
	}

	const float angle_max = math::min(math::radians(_aero_angle_k / (speed * speed)), M_PI_F);
	const Vector3f v_dir = _vel / speed;
	const Vector3f nose = -body_z; // thrust direction
	const float cos_angle = math::constrain(nose.dot(v_dir), -1.f, 1.f);

	if (acosf(cos_angle) <= angle_max) {
		return false;
	}

	/* Braking: thrust pointing more than 90 deg away from the velocity. At
	 * moderate speed let it through unlimited so the nose snaps through the
	 * dangerous band quickly and flares. Same log, 116 s, stick released at
	 * 38 m/s: nose went from 37 to 97 deg off the velocity in 0.6 s, motors
	 * touched 0 for ~0.2 s, then sat balanced (fin moment is small near 90 deg,
	 * the fins are stalled/broadside) and the vehicle braked 38 -> 17 m/s in
	 * 2.5 s holding altitude within 0.5 m. Above MPC_TS_BRK_SPD the limit stays
	 * on and speed is bled with the nose near the flight path instead. */
	if ((cos_angle < 0.f) && (speed < _aero_brake_speed)) {
		return false;
	}

	// direction, perpendicular to the velocity, in which the nose currently deviates
	Vector3f perp = nose - v_dir * cos_angle;

	if (perp.norm() < 1e-3f) {
		// nose (anti-)parallel to velocity: deviate towards up instead
		perp = Vector3f(0.f, 0.f, -1.f) - v_dir * (-v_dir(2));

		if (perp.norm() < 1e-3f) {
			return false; // vertical flight, nothing sensible to do
		}
	}

	perp.normalize();
	body_z = -(v_dir * cosf(angle_max) + perp * sinf(angle_max));
	return true;
}

/* [2026-09 trinidrone] Zoom brake above MPC_TS_BRK_SPD with a centred or
 * backward demand.
 *
 * log_6_2026-9-28-00-52-51.ulg:
 *   - Neutral stick at speed with the thrust at MPC_TS_THR_FLR (0.40): speed
 *     only settled where that thrust balances drag, 79.4 m/s, sinking 2 m/s,
 *     and the nose could not be held (motors 0.67/0.0x/0.67/0.00, tilt 87-90
 *     against 78-85). At 78 m/s during the acceleration the same angle of
 *     attack took only 0.84/0.65/0.84/0.64 at collective 0.74 - at this speed
 *     the airframe needs a high collective to be controllable at all, and a
 *     high collective along the nose means forward thrust. Thrust alone
 *     cannot slow it down.
 *   - What did slow it down: at 100 m/s lift exceeded weight at the tilt
 *     limit and the vehicle zoomed, 100 -> 84 m/s and +90 m in 6 s, with
 *     rates ~0 and motors well inside their range (collective 0.87 -> 0.44).
 *
 * So brake by trading speed for height, the way the airframe already does it
 * on its own: hold the thrust at MPC_TS_THR_BRK (authority), put the nose
 * MPC_TS_BRK_AOA above the flight path in the vertical plane of the velocity.
 * Lift bends the path up, the nose follows the path, speed bleeds; as the
 * path steepens the nose comes up towards vertical - the hover attitude.
 * Once horizontal speed is below MPC_TS_BRK_SPD the stock flare takes over.
 * Cost: height (up to ~(V^2 - MPC_TS_BRK_SPD^2) / 2g minus drag losses,
 * i.e. on the order of 100-200 m from 80 m/s) - attitude before altitude. */
void PositionControl::_zoomBrake(Vector3f &body_z, float &collective_thrust, const float weight)
{
	const float speed = _vel.norm();

	if (speed < 1.f) {
		return;
	}

	const Vector3f v_dir = _vel / speed;
	const Vector3f up(0.f, 0.f, -1.f);

	// unit vector perpendicular to the velocity, in its vertical plane, pointing up
	Vector3f w = up - v_dir * v_dir.dot(up);
	Vector3f nose;

	if (w.norm() < 1e-3f) {
		nose = up; // already going straight up

	} else {
		w.normalize();
		const float aoa = math::radians(_zoom_brake_aoa);
		nose = v_dir * cosf(aoa) + w * sinf(aoa);
	}

	// never point the thrust below the horizon
	if (nose(2) > 0.f) {
		nose(2) = 0.f;
		nose.normalize();
	}

	/* [2026-09 trinidrone] Pitch up first, throttle down second.
	 * MPC_TS_THR_BRK = 0.40/0.50 from the start: nose could not be raised at
	 * speed, no deceleration, attitude lost. MPC_TS_THR_BRK = 0.60
	 * (log_2_2026-9-28-01-24-22): clean pull-up (tilt error ~1 deg, motors
	 * 0.78/0.42), but once the path was steep 0.60 > hover (0.48) kept
	 * accelerating the climb - 46 m/s up, +556 m, total speed only 95 -> 60.
	 * So: full MPC_TS_THR_BRK while the path is still shallow (that is where
	 * the fins resist the nose-up), then ramp to MPC_TS_THR_BRK2 (below hover)
	 * as the flight path angle goes from ZOOM_GAMMA_START to ZOOM_GAMMA_END -
	 * by then the nose sits on a steep path and the fins only have to be held
	 * at a small angle, while gravity plus the lower thrust bleed the climb. */
	const float gamma = asinf(math::constrain(-_vel(2) / speed, -1.f, 1.f)); // flight path angle, + = climbing
	const float ramp = math::constrain((gamma - ZOOM_GAMMA_START) / (ZOOM_GAMMA_END - ZOOM_GAMMA_START), 0.f, 1.f);
	const float thrust_hi = math::constrain(_zoom_brake_thr, _lim_thr_min, _lim_thr_max);
	const float thrust_lo = math::constrain(_zoom_brake_thr_lo, _lim_thr_min, thrust_hi);
	const float thrust = thrust_hi + (thrust_lo - thrust_hi) * ramp;

	body_z = Vector3f(body_z * (1.f - weight) + (-nose) * weight).normalized();
	collective_thrust = (1.f - weight) * collective_thrust + weight * (-thrust);
	_speed_mode_weight = weight;
	_zoom_brake_active = true;
}
/************************************************************************************/

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
