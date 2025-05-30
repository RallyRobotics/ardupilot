/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   AP_Landing.cpp - Landing logic handler for ArduPlane
 */

#include "AP_Landing.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_InternalError/AP_InternalError.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_Landing::var_info[] = {

    // @Param: SLOPE_RCALC
    // @DisplayName: Landing slope re-calc threshold
    // @Description: This parameter is used when using a rangefinder during landing for altitude correction from baro drift (RNGFND_LANDING=1) and the altitude correction indicates your altitude is lower than the intended slope path. This value is the threshold of the correction to re-calculate the landing approach slope. Set to zero to keep the original slope all the way down and any detected baro drift will be corrected by pitching/throttling up to snap back to resume the original slope path. Otherwise, when a rangefinder altitude correction exceeds this threshold it will trigger a slope re-calculate to give a shallower slope. This also smoothes out the approach when flying over objects such as trees. Recommend a value of 2m.
    // @Range: 0 5
    // @Units: m
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("SLOPE_RCALC", 1, AP_Landing, slope_recalc_shallow_threshold, 2.0f),

    // @Param: ABORT_DEG
    // @DisplayName: Landing auto-abort slope threshold
    // @Description: This parameter is used when using a rangefinder during landing for altitude correction from baro drift (RNGFND_LANDING=1) and the altitude correction indicates your actual altitude is higher than the intended slope path. Normally it would pitch down steeply but that can result in a crash with high airspeed so this allows remembering the baro offset and self-abort the landing and come around for another landing with the correct baro offset applied for a perfect slope. An auto-abort go-around will only happen once, next attempt will not auto-abort again. This operation happens entirely automatically in AUTO mode. This value is the delta degrees threshold to trigger the go-around compared to the original slope. Example: if set to 5 deg and the mission planned slope is 15 deg then if the new slope is 21 then it will go-around. Set to 0 to disable. Requires LAND_SLOPE_RCALC > 0.
    // @Range: 0 90
    // @Units: deg
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ABORT_DEG", 2, AP_Landing, slope_recalc_steep_threshold_to_abort, 0),

    // @Param: PITCH_DEG
    // @DisplayName: Landing Pitch
    // @Description: Used in autoland to give the minimum pitch in the final stage of landing (after the flare). This parameter can be used to ensure that the final landing attitude is appropriate for the type of undercarriage on the aircraft. Note that it is a minimum pitch only - the landing code will control pitch above this value to try to achieve the configured landing sink rate.
    // @Units: deg
    // @Range: -20 20
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("PITCH_DEG", 3, AP_Landing, pitch_deg, 0),

    // @Param: FLARE_ALT
    // @DisplayName: Landing flare altitude
    // @Description: Altitude in autoland at which to lock heading and flare to the LAND_PITCH_DEG pitch. Note that this option is secondary to LAND_FLARE_SEC. For a good landing it preferable that the flare is triggered by LAND_FLARE_SEC.
    // @Units: m
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FLARE_ALT", 4, AP_Landing, flare_alt, 3.0f),

    // @Param: FLARE_SEC
    // @DisplayName: Landing flare time
    // @Description: Vertical time before landing point at which to lock heading and flare with the motor stopped. This is vertical time, and is calculated based solely on the current height above the ground and the current descent rate.  Set to 0 if you only wish to flare based on altitude (see LAND_FLARE_ALT).
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FLARE_SEC", 5, AP_Landing, flare_sec, 2.0f),

    // @Param: PF_ALT
    // @DisplayName: Landing pre-flare altitude
    // @Description: Altitude to trigger pre-flare flight stage where LAND_PF_ARSPD controls airspeed. The pre-flare flight stage trigger works just like LAND_FLARE_ALT but higher. Disabled when LAND_PF_ARSPD is 0.
    // @Units: m
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PF_ALT", 6, AP_Landing, pre_flare_alt, 10.0f),

    // @Param: PF_SEC
    // @DisplayName: Landing pre-flare time
    // @Description: Vertical time to ground to trigger pre-flare flight stage where LAND_PF_ARSPD controls airspeed. This pre-flare flight stage trigger works just like LAND_FLARE_SEC but earlier. Disabled when LAND_PF_ARSPD is 0.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PF_SEC", 7, AP_Landing, pre_flare_sec, 6.0f),

    // @Param: PF_ARSPD
    // @DisplayName: Landing pre-flare airspeed
    // @Description: Desired airspeed during pre-flare flight stage. This is useful to reduce airspeed just before the flare. Use 0 to disable.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PF_ARSPD", 8, AP_Landing, pre_flare_airspeed, 0),

    // @Param: THR_SLEW
    // @DisplayName: Landing throttle slew rate
    // @Description: This parameter sets the slew rate for the throttle during auto landing. When this is zero the THR_SLEWRATE parameter is used during landing. The value is a percentage throttle change per second, so a value of 20 means to advance the throttle over 5 seconds on landing. Values below 50 are not recommended as it may cause a stall when airspeed is low and you can not throttle up fast enough.
    // @Units: %
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_SLEW", 9, AP_Landing, throttle_slewrate, 0),

    // @Param: DISARMDELAY
    // @DisplayName: Landing disarm delay
    // @Description: After a landing has completed using a LAND waypoint, automatically disarm after this many seconds have passed. Use 0 to not disarm.
    // @Units: s
    // @Increment: 1
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("DISARMDELAY", 10, AP_Landing, disarm_delay, 20),

    // @Param: THEN_NEUTRL
    // @DisplayName: Set servos to neutral after landing
    // @Description: When enabled, after an autoland and auto-disarm via LAND_DISARMDELAY happens then set all servos to neutral. This is helpful when an aircraft has a rough landing upside down or a crazy angle causing the servos to strain.
    // @Values: 0:Disabled, 1:Servos to Neutral, 2:Servos to Zero PWM
    // @User: Advanced
    AP_GROUPINFO("THEN_NEUTRL", 11, AP_Landing, then_servos_neutral, 0),

    // @Param: ABORT_THR
    // @DisplayName: Landing abort using throttle
    // @Description: Allow a landing abort to trigger with an input throttle >= 90%. This works with or without stick-mixing enabled.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ABORT_THR", 12, AP_Landing, abort_throttle_enable, 0),

    // @Param: FLAP_PERCNT
    // @DisplayName: Landing flap percentage
    // @Description: The amount of flaps (as a percentage) to apply in the landing approach and flare of an automatic landing
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLAP_PERCNT", 13, AP_Landing, flap_percent, 0),

    // @Param: OPTIONS
    // @DisplayName: Landing options bitmask
    // @Description: Bitmask of options to use with landing.
    // @Bitmask: 0: honor min throttle during landing flare,1: Increase Target landing airspeed constraint From Trim Airspeed to AIRSPEED_MAX
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 16, AP_Landing, _options, 0),

    // @Param: FLARE_AIM
    // @DisplayName: Flare aim point adjustment percentage.
    // @Description: This parameter controls how much the aim point is moved to allow for the time spent in the flare manoeuvre. When set to 100% the aim point is adjusted on the assumption that the flare sink rate controller instantly achieves the sink rate set by TECS_LAND_SINK. when set to 0%, no aim point adjustment is made. If the plane consistently touches down short of the aim point reduce the parameter and vice verse.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLARE_AIM", 17, AP_Landing, flare_effectivness_pct, 50),

    // @Param: WIND_COMP
    // @DisplayName: Headwind Compensation when Landing
    // @Description: This param controls how much headwind compensation is used when landing.  Headwind speed component multiplied by this parameter is added to TECS_LAND_ARSPD command.  Set to Zero to disable.  Note:  The target landing airspeed command is still limited to AIRSPEED_MAX.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("WIND_COMP", 18, AP_Landing, wind_comp, 50),

    // @Param: TYPE
    // @DisplayName: Auto-landing type
    // @Description: Specifies the auto-landing type to use
    // @Values: 0:Standard Glide Slope, 1:Deepstall
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE",    14, AP_Landing, type, TYPE_STANDARD_GLIDE_SLOPE, AP_PARAM_FLAG_ENABLE),

#if HAL_LANDING_DEEPSTALL_ENABLED
    // @Group: DS_
    // @Path: AP_Landing_Deepstall.cpp
    AP_SUBGROUPINFO(deepstall, "DS_", 15, AP_Landing, AP_Landing_Deepstall),
#endif

    // additional global params should be placed in the list above TYPE to avoid the enable flag hiding the deepstall params

    AP_GROUPEND
};

    // constructor
AP_Landing::AP_Landing(AP_Mission &_mission, AP_AHRS &_ahrs, AP_TECS *_tecs_Controller, AP_Navigation *_nav_controller, AP_FixedWing &_aparm,
                       set_target_altitude_proportion_fn_t _set_target_altitude_proportion_fn,
                       constrain_target_altitude_location_fn_t _constrain_target_altitude_location_fn,
                       adjusted_altitude_cm_fn_t _adjusted_altitude_cm_fn,
                       adjusted_relative_altitude_cm_fn_t _adjusted_relative_altitude_cm_fn,
                       disarm_if_autoland_complete_fn_t _disarm_if_autoland_complete_fn,
                       update_flight_stage_fn_t _update_flight_stage_fn) :
    mission(_mission)
    ,ahrs(_ahrs)
    ,tecs_Controller(_tecs_Controller)
    ,nav_controller(_nav_controller)
    ,aparm(_aparm)
    ,set_target_altitude_proportion_fn(_set_target_altitude_proportion_fn)
    ,constrain_target_altitude_location_fn(_constrain_target_altitude_location_fn)
    ,adjusted_altitude_cm_fn(_adjusted_altitude_cm_fn)
    ,adjusted_relative_altitude_cm_fn(_adjusted_relative_altitude_cm_fn)
    ,disarm_if_autoland_complete_fn(_disarm_if_autoland_complete_fn)
    ,update_flight_stage_fn(_update_flight_stage_fn)
#if HAL_LANDING_DEEPSTALL_ENABLED
    ,deepstall(*this)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  return a location alt in cm as AMSL
  assumes loc frame is either AMSL or ABOVE_TERRAIN
*/
int32_t AP_Landing::loc_alt_AMSL_cm(const Location &loc) const
{
    int32_t alt_cm;
    // try first with full conversion
    if (loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        return alt_cm;
    }
    if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        // if we can't get true terrain then assume flat terrain
        // around home
        return loc.alt + ahrs.get_home().alt;
    }

    // this should not happen, but return a value
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return loc.alt;
}

void AP_Landing::do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude)
{
    Log(); // log old state so we get a nice transition from old to new here

    flags.commanded_go_around = false;

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_do_land(cmd, relative_altitude);
        break;
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        deepstall.do_land(cmd, relative_altitude);
        break;
#endif
    default:
        // a incorrect type is handled in the verify_land
        break;
    }

    Log();
}

/*
  update navigation for landing. Called when on landing approach or
  final flare
 */
bool AP_Landing::verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
        const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms, const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range)
{
    bool success = true;

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        success = type_slope_verify_land(prev_WP_loc, next_WP_loc, current_loc,
                height, sink_rate, wp_proportion, last_flying_ms, is_armed, is_flying, rangefinder_state_in_range);
        break;
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        success = deepstall.verify_land(prev_WP_loc, next_WP_loc, current_loc,
                                             height, sink_rate, wp_proportion, last_flying_ms, is_armed, is_flying, rangefinder_state_in_range);
        break;
#endif
    default:
        // returning TRUE while executing verify_land() will increment the
        // mission index which in many cases will trigger an RTL for end-of-mission
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Landing configuration error, invalid LAND_TYPE");
        success = true;
        break;
    }
    Log();
    return success;
}

bool AP_Landing::verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
    const int32_t auto_state_takeoff_altitude_rel_cm, bool &throttle_suppressed)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_verify_abort_landing(prev_WP_loc, next_WP_loc, throttle_suppressed);
        break;
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        deepstall.verify_abort_landing(prev_WP_loc, next_WP_loc, throttle_suppressed);
        break;
#endif
    default:
        break;
    }

    // see if we have reached abort altitude
     if (adjusted_relative_altitude_cm_fn() > auto_state_takeoff_altitude_rel_cm) {
         next_WP_loc = current_loc;
         mission.stop();
         if (restart_landing_sequence()) {
             mission.resume();
         }
         // else we're in AUTO with a stopped mission and handle_auto_mode() will set RTL
#if AP_FENCE_ENABLED
        AC_Fence *fence = AP::fence();
        if (fence) {
            fence->auto_enable_fence_after_takeoff();
        }
#endif
     }

     Log();

     // make sure to always return false so it leaves the mission index alone
     return false;
}

void AP_Landing::adjust_landing_slope_for_rangefinder_bump(AP_FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, const float wp_distance, int32_t &target_altitude_offset_cm)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_adjust_landing_slope_for_rangefinder_bump(rangefinder_state, prev_WP_loc, next_WP_loc, current_loc, wp_distance, target_altitude_offset_cm);
        break;
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
#endif
    default:
        break;
    }
}

// send out any required mavlink messages
bool AP_Landing::send_landing_message(mavlink_channel_t chan) {
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.send_deepstall_message(chan);
#endif
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}

bool AP_Landing::is_flaring(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_flaring();
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
#endif
    default:
        return false;
    }
}

// return true while the aircraft is performing a landing approach
// when true the vehicle will:
//   - disable ground steering
//   - call setup_landing_glide_slope() and adjust_landing_slope_for_rangefinder_bump()
//   - will be considered flying if sink rate > 0.2, and can trigger crash detection
bool AP_Landing::is_on_approach(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_on_approach();
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.is_on_approach();
#endif
    default:
        return false;
    }
}

// return true while the aircraft is allowed to perform ground steering
bool AP_Landing::is_ground_steering_allowed(void) const
{
    if (!flags.in_progress) {
        return true;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_on_approach();
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return false;
#endif
    default:
        return true;
    }
}

// return true when at the last stages of a land when an impact with the ground is expected soon
// when true is_flying knows that the vehicle was expecting to stop flying, possibly because of a hard impact
bool AP_Landing::is_expecting_impact(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_expecting_impact();
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
#endif
    default:
        return false;
    }
}

// returns true when the landing library has overriden any servos
bool AP_Landing::override_servos(void) {
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.override_servos();
#endif
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}

// returns a PID_Info object if there is one available for the selected landing
// type, otherwise returns a nullptr, indicating no data to be logged/sent
const AP_PIDInfo* AP_Landing::get_pid_info(void) const
{
    switch (type) {
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return &deepstall.get_pid_info();
#endif
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return nullptr;
    }
}

/*
  a special glide slope calculation for the landing approach

  During the land approach use a linear glide slope to a point
  projected through the landing point. We don't use the landing point
  itself as that leads to discontinuities close to the landing point,
  which can lead to erratic pitch control
 */

void AP_Landing::setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_setup_landing_glide_slope(prev_WP_loc, next_WP_loc, current_loc, target_altitude_offset_cm);
        break;
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
#endif
    default:
        break;
    }
}

/*
     Restart a landing by first checking for a DO_LAND_START and
     jump there. Otherwise decrement waypoint so we would re-start
     from the top with same glide slope. Return true if successful.
 */
bool AP_Landing::restart_landing_sequence()
{
    if (mission.get_current_nav_cmd().id != MAV_CMD_NAV_LAND) {
        return false;
    }

    uint16_t do_land_start_index = 0;
    Location loc;
    if (ahrs.get_location(loc)) {
        do_land_start_index = mission.get_landing_sequence_start(loc);
    }
    uint16_t prev_cmd_with_wp_index = mission.get_prev_nav_cmd_with_wp_index();
    bool success = false;
    uint16_t current_index = mission.get_current_nav_index();
    AP_Mission::Mission_Command cmd;

    if (mission.read_cmd_from_storage(current_index+1,cmd) &&
            cmd.id == MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT &&
            (cmd.p1 == 0 || cmd.p1 == 1) &&
            mission.set_current_cmd(current_index+1))
    {
        // if the next immediate command is MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT to climb, do it
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Restarted landing sequence. Climbing to %dm", (signed)cmd.content.location.alt/100);
        success =  true;
    }
    else if (do_land_start_index != 0 &&
            mission.set_current_cmd(do_land_start_index))
    {
        // look for a DO_LAND_START and use that index
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Restarted landing via DO_LAND_START: %d",do_land_start_index);
        success =  true;
    }
    else if (prev_cmd_with_wp_index != AP_MISSION_CMD_INDEX_NONE &&
               mission.set_current_cmd(prev_cmd_with_wp_index))
    {
        // if a suitable navigation waypoint was just executed, one that contains lat/lng/alt, then
        // repeat that cmd to restart the landing from the top of approach to repeat intended glide slope
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Restarted landing sequence at waypoint %d", prev_cmd_with_wp_index);
        success =  true;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unable to restart landing sequence");
        success =  false;
    }

    if (success) {
        // exit landing stages if we're no longer executing NAV_LAND
        update_flight_stage_fn();
    }

    Log();
    return success;
}

int32_t AP_Landing::constrain_roll(const int32_t desired_roll_cd, const int32_t level_roll_limit_cd)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_constrain_roll(desired_roll_cd, level_roll_limit_cd);
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
#endif
    default:
        return desired_roll_cd;
    }
}

// returns true if landing provided a Location structure with the current target altitude
bool AP_Landing::get_target_altitude_location(Location &location)
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.get_target_altitude_location(location);
#endif
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}

/*
 * returns target airspeed in cm/s depending on flight stage
 */
int32_t AP_Landing::get_target_airspeed_cm(void)
{
    if (!flags.in_progress) {
        // not landing, use regular cruise airspeed
        return aparm.airspeed_cruise*100;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_get_target_airspeed_cm();
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.get_target_airspeed_cm();
#endif
    default:
        // don't return the landing airspeed, because if type is invalid we have
        // no postive indication that the land airspeed has been configured or
        // how it was meant to be utilized
        return tecs_Controller->get_target_airspeed();
    }
}

/*
 * request a landing abort given the landing type
 * return true on success
 */
bool AP_Landing::request_go_around(void)
{
    bool success = false;

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        success = type_slope_request_go_around();
        break;
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        success = deepstall.request_go_around();
        break;
#endif
    default:
        break;
    }

    Log();
    return success;
}

void AP_Landing::handle_flight_stage_change(const bool _in_landing_stage)
{
    Log(); // log old value to plot discrete transitions
    flags.in_progress = _in_landing_stage;
    flags.commanded_go_around = false;
    Log();
}

/*
 * returns true when a landing is complete, usually used to disable throttle
 */
bool AP_Landing::is_complete(void) const
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_complete();
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return false;
#endif
    default:
        return true;
    }
}

void AP_Landing::Log(void) const
{
#if HAL_LOGGING_ENABLED
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_log();
        break;
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        deepstall.Log();
        break;
#endif
    default:
        break;
    }
#endif
}

/*
 * returns true when throttle should be suppressed while landing
 */
bool AP_Landing::is_throttle_suppressed(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_throttle_suppressed();
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.is_throttle_suppressed();
#endif
    default:
        return false;
    }
}

//defaults to false, but _options bit zero enables it.
bool AP_Landing::use_thr_min_during_flare(void) const
{
    return (OptionsMask::ON_LANDING_FLARE_USE_THR_MIN & _options) != 0;
}

//defaults to false, but _options bit zero enables it.
bool AP_Landing::allow_max_airspeed_on_land(void) const
{
    return (OptionsMask::ON_LANDING_USE_ARSPD_MAX & _options) != 0;
}

/*
 * returns false when the vehicle might not be flying forward while landing
 */
bool AP_Landing::is_flying_forward(void) const
{
    if (!flags.in_progress) {
        return true;
    }

    switch (type) {
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.is_flying_forward();
#endif
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return true;
    }
}

/*
 * attempt to terminate flight with an immediate landing
 * returns true if the landing library can and is terminating the landing
 */
bool AP_Landing::terminate(void) {
    switch (type) {
#if HAL_LANDING_DEEPSTALL_ENABLED
    case TYPE_DEEPSTALL:
        return deepstall.terminate();
#endif
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}

/*
  run parameter conversions
 */
void AP_Landing::convert_parameters(void)
{
    // added January 2024
    pitch_deg.convert_centi_parameter(AP_PARAM_INT16);
}
