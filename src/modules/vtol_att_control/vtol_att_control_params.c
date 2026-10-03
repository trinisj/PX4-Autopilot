/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
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
 * @file vtol_att_control_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Sander Smeets <sander@droneslab.com>
 */

/**
 * VTOL Type (Tailsitter=0, Tiltrotor=1, Standard=2)
 *
 * @value 0 Tailsitter
 * @value 1 Tiltrotor
 * @value 2 Standard
 * @min 0
 * @max 2
 * @reboot_required true
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TYPE, 0);

/**
 * Lock control surfaces in hover
 *
 * If set to 1 the control surfaces are locked at the disarmed value in multicopter mode.
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_ELEV_MC_LOCK, 1);

/**
 * Duration of a front transition
 *
 * Time in seconds used for a transition
 *
 * @unit s
 * @min 0.1
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_DUR, 5.0f);

/**
 * Maximum duration of a back transition
 *
 * Transition is also declared over if the groundspeed drops below MPC_XY_CRUISE.
 *
 * @unit s
 * @min 0.1
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_DUR, 10.0f);

/**
 * Target throttle value for the transition to fixed-wing flight.
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_THR, 1.0f);

/**
 * Approximate deceleration during back transition
 *
 * Used to calculate back transition distance in an auto mode.
 * For standard vtol and tiltrotors a controller is used to track this value during the transition.
 *
 * @unit m/s^2
 * @min 0.5
 * @max 10
 * @increment 0.1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_MSS, 2.0f);

/**
 * Transition blending airspeed
 *
 * Airspeed at which we can start blending both fw and mc controls. Set to 0 to disable.
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_BLEND, 8.0f);

/**
 * Transition airspeed
 *
 * Airspeed at which we can switch to fw mode
 *
 * @unit m/s
 * @min 0.00
 * @max 100.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_TRANS, 10.0f);

/**
 * Front transition timeout
 *
 * Time in seconds after which transition will be cancelled.
 *
 * @unit s
 * @min 0.1
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_TIMEOUT, 15.0f);

/**
 * Front transition minimum time
 *
 * Minimum time in seconds for front transition.
 *
 * @unit s
 * @min 0.0
 * @max 20.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_MIN_TM, 2.0f);

/**
 * Quad-chute altitude
 *
 * Minimum altitude for fixed-wing flight. When the vehicle is in fixed-wing mode
 * and the altitude drops below this altitude (relative altitude above local origin),
 * it will instantly switch back to MC mode and execute behavior defined in COM_QC_ACT.
 *
 * @unit m
 * @min 0.0
 * @max 200.0
 * @increment 1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_MIN_ALT, 0.0f);

/**
 * Quad-chute uncommanded descent threshold
 *
 * Altitude error threshold for quad-chute triggering during fixed-wing flight.
 * The check is only active if altitude is controlled and the vehicle is below the current altitude reference.
 * The altitude error is relative to the highest altitude the vehicle has achieved since it has flown below the current
 * altitude reference.
 *
 * Set to 0 do disable.
 *
 * @unit m
 * @min 0.0
 * @max 200.0
 * @increment 1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_QC_ALT_LOSS, 0.0f);

/**
 * Quad-chute transition altitude loss threshold
 *
 * Altitude loss threshold for quad-chute triggering during VTOL transition to fixed-wing flight
 * in altitude-controlled flight modes.
 * Active until 5s after completing transition to fixed-wing.
 * If the current altitude is more than this value below the altitude at the beginning of the
 * transition, it will instantly switch back to MC mode and execute behavior defined in COM_QC_ACT.
 *
 * Set to 0 do disable this threshold.
 *
 * @unit m
 * @min 0
 * @max 50
 * @increment 1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_QC_T_ALT_LOSS, 20.0f);

/**
 * Quad-chute max pitch threshold
 *
 * Absolute pitch threshold for quad-chute triggering in FW mode.
 * Above this the vehicle will transition back to MC mode and execute behavior defined in COM_QC_ACT.
 * Set to 0 do disable this threshold.
 *
 * @unit deg
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_P, 0);

/**
 * Quad-chute max roll threshold
 *
 * Absolute roll threshold for quad-chute triggering in FW mode.
 * Above this the vehicle will transition back to MC mode and execute behavior defined in COM_QC_ACT.
 * Set to 0 do disable this threshold.
 *
 * @unit deg
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_R, 0);

/**
 * Quad-chute maximum height
 *
 * Maximum height above the ground (if available, otherwise above
 * Home if available, otherwise above the local origin) where triggering a quad-chute is possible.
 * At high altitudes there is a big risk to deplete the battery and therefore crash if quad-chuting there.
 *
 * @unit m
 * @min 0
 * @increment 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_HMAX, 0);

/**
 * Airspeed-less front transition time (open loop)
 *
 * The duration of the front transition when there is no airspeed feedback available.
 * When airspeed is used, transition timeout is declared if airspeed does not
 * reach VT_ARSP_BLEND after this time.
 *
 * @unit s
 * @min 1.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TR_OL_TM, 6.0f);

// 261003
/**
 * Front transition maximum tolerated sink rate before holding tilt progress
 *
 * [2026-09 custom] The stock front-transition tilt setpoint advances purely
 * on elapsed time (VT_F_TRANS_DUR), reaching close to 90 deg regardless of
 * whether the airframe actually has enough airspeed/lift yet to be tilted
 * that far - at full tilt, multicopter thrust has almost no vertical
 * component left, so an airframe that generates little lift at low speed
 * (e.g. no wing, small fin-only surfaces needing high speed for meaningful
 * lift) can be committed to a tilt it cannot yet support while it waits for
 * VT_ARSP_TRANS to be reached, which shows up as an uncontrolled sink.
 *
 * This parameter bounds that: while in the front transition, if the
 * measured climb rate (vehicle_local_position.vz, NED, positive = sinking)
 * exceeds this value, further tilt progress is held (not reversed) until
 * the sink rate comes back under the limit. This makes the tilt schedule
 * respond to the airframe's actual, currently measured lift/thrust
 * performance instead of only to a pre-set duration - conceptually the
 * same judgement a pilot makes flying the transition by feel, but driven by
 * the real climb-rate measurement each cycle rather than a fixed number
 * picked in advance.
 *
 * Set higher than the sink rate seen during ordinary, healthy front
 * transitions on this airframe (check tecs_status / vehicle_local_position
 * logs from a transition that completed cleanly) so it does not fire on
 * normal transition sink; set low enough that it still catches a genuine
 * "falling, not flying" situation before significant altitude is lost.
 * VT_F_TRANS_DUR and VT_TRANS_TIMEOUT still bound the overall transition -
 * this only slows tilt progress within that budget, it does not remove
 * those limits.
 *
 * @unit m/s
 * @min 0.5
 * @max 20.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TR_SINK_MAX, 3.0f);

/**
 * Back transition nose-up speed gate (tailsitter)
 *
 * [2026-09 custom] HANDOFF 9: symmetric to VT_F_TR_SINK_MAX, but for the
 * back transition. Stock advances the nose-up schedule purely on
 * _time_since_trans_start, regardless of speed. On this airframe, above
 * roughly this speed the fins' aerodynamic restoring moment overpowers the
 * MC differential-thrust pitch authority: commanding nose-up does not
 * raise the actual nose (log_6_2026-9-27-18-52-50, log_3_2026-9-27-22-54-28
 * back transitions starting at 90+ m/s: commanded nose 7 -> 52 deg, actual
 * nose -14 deg and falling, motors pinned 1.0/0.0, roll/altitude both
 * unstable while this continues), and only wastes the pitch fight while
 * the vehicle is also not decelerating any faster for it.
 *
 * While the measured groundspeed (airspeed if available upstream) is above
 * this value, the nose-up schedule is held at its start position instead
 * of advancing - the vehicle keeps its cruise-like attitude and decelerates
 * on drag alone. Once speed drops below this value the schedule resumes
 * and the nose comes up for real. This trades transition distance/time
 * (and some altitude, since MC altitude-hold may climb while decelerating
 * at this attitude) for a nose-up attempt the airframe can actually make
 * good on. VT_B_TRANS_DUR/VT_TRANS_TIMEOUT still bound the overall back
 * transition.
 *
 * Set from the speed at which back transitions on this airframe are
 * observed to actually raise the nose (5.2/8.3: ~60 m/s). Position/velocity
 * data unavailable is treated as "still fast" (fail safe: hold, don't
 * raise blind).
 *
 * @unit m/s
 * @min 5.0
 * @max 100.0
 * @increment 1.0
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TR_SPD_GATE, 60.0f);

/**
 * Front transition minimum measured lift fraction (tailsitter)
 *
 * [2026-09 custom] Front transition to fixed-wing only completes once the
 * measured aerodynamic lift, as a fraction of weight, reaches this value
 * (in addition to the pitch and VT_ARSP_TRANS conditions). Lift is
 * estimated each cycle from the vertical force balance:
 *   lift/W = 1 + a_up/g - (thrust / hover_thrust) * cos(tilt)
 * low-pass filtered with a 0.5 s time constant.
 *
 * Hover reads ~0. Tune from the "front transition complete" log message
 * and from logs of transitions that went well.
 *
 * Set to 0 to disable (stock behaviour for this condition).
 *
 * @min 0.0
 * @max 1.5
 * @increment 0.05
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TR_LIFT_MIN, 0.7f);
/***********************************************************************/

/**
 * Differential thrust in forwards flight.
 *
 * Enable differential thrust seperately for roll, pitch, yaw in forward (fixed-wing) mode.
 * The effectiveness of differential thrust around the corresponding axis can be
 * tuned by setting VT_FW_DIFTHR_S_R / VT_FW_DIFTHR_S_P / VT_FW_DIFTHR_S_Y.
 *
 * @min 0
 * @max 7
 * @bit 0 Yaw
 * @bit 1 Roll
 * @bit 2 Pitch
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_DIFTHR_EN, 0);

/**
 * Roll differential thrust factor in forward flight
 *
 * Differential thrust in forward flight is enabled via VT_FW_DIFTHR_EN.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_S_R, 1.f);

/**
 * Pitch differential thrust factor in forward flight
 *
 * Differential thrust in forward flight is enabled via VT_FW_DIFTHR_EN.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_S_P, 1.f);

/**
 * Yaw differential thrust factor in forward flight
 *
 * Differential thrust in forward flight is enabled via VT_FW_DIFTHR_EN.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_S_Y, 0.1f);

/**
 * Backtransition deceleration setpoint to pitch I gain.
 *
 * @unit rad s/m
 * @min 0
 * @max 0.3
 * @decimal 2
 * @increment 0.05
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_I, 0.1f);

/**
 * Minimum pitch angle during hover.
 *
 * Any pitch setpoint below this value is translated to a forward force by the fixed-wing forward actuation if
 * VT_FWD_TRHUST_EN is set.
 *
 * @unit deg
 * @min -10.0
 * @max 45.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_MIN, -5.0f);

/**
 * Minimum pitch angle during hover landing.
 *
 * Overrides VT_PITCH_MIN when the vehicle is in LAND mode (hovering).
 * During landing it can be beneficial to reduce the pitch angle to reduce the generated lift in head wind.
 *
 * @unit deg
 * @min -10.0
 * @max 45.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_LND_PITCH_MIN, -5.0f);

/**
 * Spoiler setting while landing (hover)
 *
 * @unit norm
 * @min -1
 * @max 1
 * @decimal 1
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_SPOILER_MC_LD, 0.f);
