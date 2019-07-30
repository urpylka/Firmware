/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file precland_params.c
 *
 * Parameters for precision landing.
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

/**
 * Landing Target Timeout
 *
 * Time after which the landing target is considered lost without any new measurements.
 *
 * @unit s
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_BTOUT, 5.0f);

/**
 * Horizontal acceptance radius
 *
 * Start descending if closer above landing target than this.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_HACC_RAD, 0.2f);

/**
 * Final approach altitude
 *
 * Allow final approach (without horizontal positioning) if losing landing target closer than this to the ground.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_FAPPR_ALT, 0.1f);

/**
 * Search altitude
 *
 * Altitude above home to which to climb when searching for the landing target.
 *
 * @unit m
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_ALT, 10.0f);

/**
 * Search timeout
 *
 * Time allowed to search for the landing target before falling back to normal landing.
 *
 * @unit s
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_TOUT, 10.0f);

/**
 * Maximum number of search attempts
 *
 * Maximum number of times to seach for the landing target if it is lost during the precision landing.
 *
 * @min 0
 * @max 100
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_MAX_SRCH, 3);

/**
 * Strict precision land
 *
 * Keep the vehicle inside a cylinder or a funnel while descending above the target.
 *
 * @boolean
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_STRICT, 0);

/**
 * Strict precision land funnel top radius
 *
 * Funnel top radius at the search altitude. Set to 0 to switch a precision land to the cylinder
 * precision land mode.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_FUNNEL_TR, 0);

/**
 * Strict precision land funnel linear part end altitude
 *
 * Funnel linear part ends at this altitude and the horizontal acceptance radius cylinder starts.
 * It's better to keep this parameter slightly greater than the final approach altitude.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_FUNNEL_LEA, 0);

/**
 * Strict precision land timeout in any state.
 *
 * A maximum time to spend in any state before precland switches to the fallback state.
 *
 * @unit s
 * @min 1.0
 * @max 600
 * @decimal 2
 * @increment 0.5
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_STATE_TIME, 10);

/**
 * Send precland info using MAVLink
 *
 * Send MAVLink status texts about the precision land procedure.
 *
 * @boolean
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_INFO, 0);

/**
 * Precision land fallback action
 *
 * An action to do if the precision land has failed.
 *
 * @min 0
 * @max 1
 * @value 0 Land
 * @value 1 RTL
 * @value 2 Safety point
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_FOAC, 0);

/**
 * Precision land active search enabled
 *
 * Search the landing target actively if the target is not visible after regular search.
 *
 * @boolean
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_ASEN, 0);

/**
 * Precision land active search limiting radius
 *
 * Limiting active search radius.
 *
 * @unit m
 * @min 0.5
 * @max 15
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_ASLR, 2);

/**
 * Precision land active search acceptance radius
 *
 * An active search setpoints active radius.
 *
 * @unit m
 * @min 0.1
 * @max 1.5
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_ASAR, 0.2);

/**
 * Precision land active search concentric cirlces radius step
 *
 * An active search step between concentric circles.
 *
 * @unit m
 * @min 0.5
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_ASCS, 1);

/**
 * Precision land active search setpoint step
 *
 * An active search step between setpoints in a circle.
 *
 * @unit m
 * @min 0.2
 * @max 3
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_ASPS, 0.2);

/**
 * Maximum number of active search search attempts
 *
 * Maximum number of times to search for the landing target actively if it is lost during the precision landing.
 *
 * @min 0
 * @max 100
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_MAX_ASRCH, 2);
