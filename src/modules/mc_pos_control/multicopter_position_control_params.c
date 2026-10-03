/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Vertical thrust required to hover
 *
 * Mapped to center throttle stick in Stabilized mode (see MPC_THR_CURVE).
 * Used for initialization of the hover thrust estimator (see MPC_USE_HTE).
 * The estimated hover thrust is used as base for zero vertical acceleration in altitude control.
 * The hover thrust is important for land detection to work correctly.
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.5f);

/**
 * Use hover thrust estimate for altitude control
 *
 * Disable to use the fixed parameter MPC_THR_HOVER instead of the hover thrust estimate in the position controller.
 * This parameter does not influence Stabilized mode throttle curve (see MPC_THR_CURVE).
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_USE_HTE, 1);

/**
 * Horizontal thrust margin
 *
 * Margin that is kept for horizontal control when higher priority vertical thrust is saturated.
 * To avoid completely starving horizontal control with high vertical error.
 *
 * @unit norm
 * @min 0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_XY_MARG, 0.3f);

/**
 * Velocity low pass cutoff frequency
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_LP, 0.0f);

/**
 * Velocity notch filter frequency
 *
 * The center frequency for the 2nd order notch filter on the velocity.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_NF_FRQ, 0.0f);

/**
 * Velocity notch filter bandwidth
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_NF_BW, 5.0f);

/**
 * Velocity derivative low pass cutoff frequency
 *
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VELD_LP, 5.0f);

/**
 * Tailsitter MC-mode aerodynamic angle limit
 *
 * [2026-09 trinidrone] For a wingless tailsitter flown in multicopter mode at
 * speed. Limits the angle between the thrust axis (the nose) and the velocity
 * vector - angle of attack and sideslip together - to
 *
 *     angle_max [deg] = MPC_TS_AOA_K / V^2      (V = ground speed, m/s)
 *
 * Unit of MPC_TS_AOA_K: deg * (m/s)^2 (not in the PX4 unit list, so not declared).
 *
 * so that angle x dynamic pressure, and with it the fins' moment pulling the
 * nose onto the flight path, stays inside what differential thrust can hold.
 * Above the limit the thrust direction is rotated towards the velocity vector;
 * when thrust then saturates, direction is kept and altitude gives way
 * (attitude before altitude). The tilt limit MPC_TILTMAX_AIR still applies on
 * top.
 *
 * Has no effect while angle_max > the current angle, i.e. at low speed
 * (with 30000: above 90 deg below ~18 m/s; 25 m/s 48 deg, 40 m/s 19 deg,
 * 60 m/s 8 deg, 90 m/s 3.7 deg).
 *
 * Reference from SITL logs: 91 m/s at 4.7 deg (~40000) flew; loss of control
 * started at 44 m/s with ~24 deg (~47000) and a collective of ~0.4.
 *
 * EXPERIMENTAL - keep at 0 for now. The collective is still set by the
 * vertical need, so a forced larger tilt also produces much more forward
 * thrust (replayed on SITL data: 0.39 -> 0.70 at 42 m/s) and pushes the
 * vehicle towards its high-speed trim regardless of the setpoint. Needs a
 * speed-management counterpart before use.
 *
 * 0 disables (stock behaviour). Only meaningful for a tailsitter in MC mode.
 *
 * @min 0
 * @max 200000
 * @decimal 0
 * @increment 1000
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TS_AOA_K, 0.0f);

/**
 * Tailsitter MC-mode brake-flare speed
 *
 * [2026-09 trinidrone] Only used with MPC_TS_AOA_K > 0. Below this speed a
 * braking command (thrust pointing more than 90 deg away from the velocity,
 * e.g. stick released in Altitude mode) bypasses the aero angle limit so the
 * nose snaps up and the vehicle flares. Above it the limit stays on and speed
 * is bled with the nose near the flight path (drag, lower thrust, or a climb).
 *
 * SITL reference: a clean flare from 38 m/s (38 -> 17 m/s in 2.5 s, altitude
 * within 0.5 m). No clean data above that yet - raise in steps.
 *
 * @unit m/s
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TS_BRK_SPD, 40.0f);

/**
 * Tailsitter MC-mode collective floor while aero-limited
 *
 * [2026-09 trinidrone] Above 15 m/s horizontal speed, keeps the MC
 * collective at least this high so differential thrust keeps range in both
 * directions; surplus thrust appears as a climb (and some extra speed).
 * Independent of MPC_TS_AOA_K. Hover, descent and landing (slow) are not
 * affected.
 *
 * SITL reference (log_12_2026-9-27-23-37-37): holding at collective 0.44,
 * loss of control once it sagged to ~0.2-0.3 with one motor at 0.
 * Suggested first value 0.40. 0 disables.
 *
 * @unit norm
 * @min 0
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TS_THR_FLR, 0.0f);

/**
 * Tailsitter MC-mode speed mode start speed
 *
 * [2026-09 trinidrone] Above this horizontal speed (blended in over the
 * next 10 m/s) the MC position controller changes roles for a tailsitter at
 * speed: the horizontal demand (pitch stick in Altitude mode, velocity loop
 * in Position mode) sets the THRUST, from MPC_TS_THR_FLR at zero demand to
 * MPC_THR_MAX at 3 g demand (= full stick), and the TILT is whatever holds
 * the altitude the vertical loop asks for with that thrust, between 45 deg
 * and MPC_TILTMAX_AIR. Pitch follows speed, throttle sets speed.
 *
 * Braking: below MPC_TS_BRK_SPD a centred/backward demand gives the stock
 * flare. Above it: zoom brake (MPC_TS_THR_BRK, MPC_TS_BRK_AOA) - speed is
 * traded for height until the flare speed is reached.
 *
 * Uses MPC_TS_THR_FLR as the minimum thrust (0 -> MPC_THR_MIN).
 * 0 disables (stock).
 *
 * @unit m/s
 * @min 0
 * @max 60
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TS_SPD_ON, 0.0f);

/**
 * Tailsitter MC-mode zoom-brake thrust
 *
 * [2026-09 trinidrone] Collective at the start of the zoom brake (speed
 * mode, horizontal speed above MPC_TS_BRK_SPD, demand centred or backwards),
 * while the flight path is still shallow (< 20 deg climb). Must be high
 * enough to raise the nose against the fins at speed: 0.40/0.50 failed in
 * SITL, 0.60 pulled up cleanly. Ramps to MPC_TS_THR_BRK2 between 20 and
 * 60 deg of climb.
 *
 * @unit norm
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TS_THR_BRK, 0.6f);

/**
 * Tailsitter MC-mode zoom-brake nose angle above the flight path
 *
 * [2026-09 trinidrone] During the zoom brake the nose is held this far above
 * the velocity vector, in its vertical plane; the resulting lift bends the
 * path up and speed is traded for height. Larger = harder pull-up, more
 * pitch moment. SITL reference: 3-5 deg held level at 80-100 m/s; lift
 * exceeded weight at ~3 deg at 100 m/s.
 *
 * @unit deg
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TS_BRK_AOA, 6.0f);

/**
 * Tailsitter MC-mode zoom-brake thrust on a steep path
 *
 * [2026-09 trinidrone] Collective the zoom brake ramps down to as the climb
 * angle goes from 20 to 60 deg. Keep it below the hover thrust so the climb
 * decelerates instead of being driven: with 0.60 all the way (> hover 0.48)
 * the SITL vehicle climbed at 46 m/s and gained 556 m while total speed only
 * fell from 95 to 60 m/s.
 *
 * @unit norm
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TS_THR_BRK2, 0.3f);
