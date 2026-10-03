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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param hover_thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }
	void resetIntegralXY() { _vel_int.xy() = matrix::Vector2f(); }

	/**
	 * If set, the tilt setpoint is computed by assuming no vertical acceleration
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * [2026-09 trinidrone] Aerodynamic angle limit (tailsitter flown in MC mode).
	 * Limits the angle between the thrust axis (= nose) and the velocity
	 * vector to  k / |v|^2  [deg], i.e. keeps (angle x dynamic pressure)
	 * roughly constant so the fins' restoring moment stays within the
	 * differential-thrust authority. k in deg*(m/s)^2, <= 0 disables (stock).
	 * See MPC_TS_AOA_K and _limitAeroAngle().
	 */
	void setAeroAngleLimit(const float k_deg_m2_s2) { _aero_angle_k = k_deg_m2_s2; }

	/**
	 * [2026-09 trinidrone] Below this speed a braking command (thrust pointing
	 * more than 90 deg away from the velocity) is passed through unlimited, so
	 * the nose can snap up and flare. See MPC_TS_BRK_SPD.
	 */
	void setAeroBrakeSpeed(const float speed) { _aero_brake_speed = speed; }

	/**
	 * [2026-09 trinidrone] Minimum collective above AERO_FLOOR_MIN_SPEED
	 * horizontal speed, to keep differential-thrust range. See MPC_TS_THR_FLR.
	 */
	void setAeroThrustFloor(const float thr) { _aero_thr_floor = thr; }

	/**
	 * [2026-09 trinidrone] Horizontal speed above which speed mode blends in
	 * (thrust from horizontal demand, tilt from vertical need). <= 0 disables.
	 * See MPC_TS_SPD_ON and _speedModeBlend().
	 */
	void setSpeedModeOn(const float speed) { _spd_mode_on = speed; }

	/**
	 * [2026-09 trinidrone] Zoom brake above MPC_TS_BRK_SPD: thrust and nose
	 * angle above the flight path. See MPC_TS_THR_BRK, MPC_TS_BRK_AOA.
	 */
	void setZoomBrake(const float thrust, const float aoa_deg) { _zoom_brake_thr = thrust; _zoom_brake_aoa = aoa_deg; }
	void setZoomBrakeThrustLow(const float thrust) { _zoom_brake_thr_lo = thrust; } ///< MPC_TS_THR_BRK2
	float speedModeWeight() const { return _speed_mode_weight; }
	bool aeroAngleLimitActive() const { return _aero_limit_active; }

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	// The range limits of the hover thrust configuration/estimate
	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid();

	void _positionControl(); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing
	bool _limitAeroAngle(matrix::Vector3f &body_z) const; ///< [2026-09 trinidrone] see setAeroAngleLimit()
	void _speedModeBlend(matrix::Vector3f &body_z, float &collective_thrust, float thrust_ned_z); ///< [2026-09 trinidrone]
	void _zoomBrake(matrix::Vector3f &body_z, float &collective_thrust, float weight); ///< [2026-09 trinidrone]

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	float _aero_angle_k{0.f}; ///< [deg*(m/s)^2] see setAeroAngleLimit(), <= 0 disables
	bool _aero_limit_active{false}; ///< aero angle limit changed the thrust direction this cycle
	bool _aero_floor_active{false}; ///< collective was raised to _aero_thr_floor this cycle
	float _aero_brake_speed{0.f}; ///< [m/s] see setAeroBrakeSpeed()
	float _aero_thr_floor{0.f}; ///< [norm] see setAeroThrustFloor(), 0 disables
	static constexpr float AERO_LIMIT_MIN_SPEED = 5.f; ///< [m/s] below this the limit is not evaluated
	static constexpr float AERO_FLOOR_MIN_SPEED = 15.f; ///< [m/s] horizontal speed above which MPC_TS_THR_FLR applies

	// [2026-09 trinidrone] speed mode, see _speedModeBlend()
	float _spd_mode_on{0.f}; ///< [m/s] MPC_TS_SPD_ON, <= 0 disables
	float _speed_mode_weight{0.f}; ///< [0,1] blend applied this cycle
	bool _spd_tilt_capped{false};
	bool _spd_thr_saturated{false};
	bool _zoom_brake_active{false};
	bool _spd_braking{false}; ///< braking state with hysteresis, see _speedModeBlend()
	bool _spd_s_sat_hi{false}; ///< along-track demand beyond full thrust
	bool _spd_s_sat_lo{false}; ///< along-track demand at/below floor thrust
	bool _vel_xy_controlled{false}; ///< horizontal velocity/position setpoint present (Position mode)
	matrix::Vector2f _spd_vel_dir{1.f, 0.f}; ///< horizontal track direction used this cycle
	static constexpr float SPEED_MODE_BRK_ENTER_POS = 3.f; ///< [m/s^2] Position mode: along-track demand below -this enters braking
	static constexpr float SPEED_MODE_BRK_LEAVE_POS = 1.f; ///< [m/s^2] Position mode: above -this leaves braking
	float _zoom_brake_thr{0.6f}; ///< [norm] MPC_TS_THR_BRK
	float _zoom_brake_aoa{6.f}; ///< [deg] MPC_TS_BRK_AOA
	float _zoom_brake_thr_lo{0.3f}; ///< [norm] MPC_TS_THR_BRK2
	static constexpr float ZOOM_GAMMA_START = 0.349f; ///< [rad] 20 deg: start ramping zoom thrust down
	static constexpr float ZOOM_GAMMA_END = 1.047f; ///< [rad] 60 deg: zoom thrust fully at MPC_TS_THR_BRK2
	static constexpr float SPEED_MODE_BLEND = 10.f; ///< [m/s] blend width above MPC_TS_SPD_ON
	static constexpr float SPEED_MODE_ACC_FULL = 3.f * 9.80665f; ///< [m/s^2] demand = full thrust (= manual stick max, log_0_2026-9-28)
	static constexpr float SPEED_MODE_ACC_MIN = 1.f; ///< [m/s^2] below this the demand counts as "centred"
	static constexpr float SPEED_MODE_TILT_MIN = 0.7854f; ///< [rad] 45 deg: steepest nose allowed in speed mode
	bool _decouple_horizontal_and_vertical_acceleration{true}; ///< Ignore vertical acceleration setpoint to remove its effect on the tilt setpoint

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */
};
