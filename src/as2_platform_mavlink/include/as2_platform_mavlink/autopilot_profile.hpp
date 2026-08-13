// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file autopilot_profile.hpp
 *
 * Autopilot-dependent behaviour of the MAVLink platform.
 *
 * Everything that differs between autopilot firmwares (flight-mode names,
 * which MAVLink setpoint fields are actually honoured, the meaning of the
 * AttitudeTarget thrust field, ...) is collected here so that the platform
 * implementation itself stays firmware agnostic.
 *
 * References:
 *  - ArduPilot Copter: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
 *  - ArduPilot flight modes: https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html
 *  - PX4 offboard: https://docs.px4.io/main/en/flight_modes/offboard.html
 *
 * @author Mohamed Elmahlawy
 */

#ifndef AS2_PLATFORM_MAVLINK__AUTOPILOT_PROFILE_HPP_
#define AS2_PLATFORM_MAVLINK__AUTOPILOT_PROFILE_HPP_

#include <string>

namespace as2_platform_mavlink
{

/**
 * @brief Autopilot firmware families supported by this platform.
 */
enum class AutopilotFamily
{
  ARDUPILOT,
  PX4
};

/**
 * @brief Meaning of the `thrust` field of mavros_msgs::msg::AttitudeTarget.
 *
 * ArduPilot Copter re-interprets that field as a *climb rate* unless the
 * GUID_OPTIONS parameter has bit 3 (value 8) set, in which case it is a
 * normalised thrust. PX4 always treats it as normalised thrust.
 */
enum class ThrustSemantics
{
  /// [0, 1] maps linearly to [0 %, 100 %] of the maximum collective thrust.
  NORMALIZED_THRUST,
  /// [0, 1] maps to [-WPNAV_SPEED_DN, +WPNAV_SPEED_UP]; 0.5 means "hold altitude".
  CLIMB_RATE
};

/**
 * @brief Vehicle types supported by this platform.
 *
 * Only multirotors (ArduCopter) are implemented: the Aerostack2 motion
 * behaviours used by this platform (takeoff, land, hover, go_to) assume a
 * vehicle able to hover and to take off vertically.
 */
enum class VehicleType
{
  COPTER
};

/**
 * @brief Firmware dependent configuration of the MAVLink platform.
 */
struct AutopilotProfile
{
  AutopilotFamily family = AutopilotFamily::ARDUPILOT;
  std::string name = "ardupilot";

  /// Flight mode used as the Aerostack2 "offboard" mode.
  std::string offboard_mode = "GUIDED";
  /// Flight mode entered when Aerostack2 requests to leave offboard control.
  std::string manual_mode = "LOITER";
  /// Flight mode entered on an EMERGENCY_HOVER alert.
  std::string hold_mode = "BRAKE";

  /// Whether SET_POSITION_TARGET_LOCAL_NED acceleration fields are honoured.
  bool supports_acceleration_setpoints = true;
  /// Whether SET_ATTITUDE_TARGET body rates are honoured (ACRO control mode).
  bool supports_body_rate_setpoints = false;
  /// Whether the vehicle needs an explicit MAV_CMD_NAV_TAKEOFF to leave the ground.
  bool requires_takeoff_command = true;
  /// Whether the offboard flight mode must be entered before arming.
  bool set_mode_before_arming = true;

  ThrustSemantics attitude_thrust = ThrustSemantics::CLIMB_RATE;

  /**
   * @brief Build the profile of a known autopilot family.
   *
   * @param autopilot_name "ardupilot" or "px4" (case insensitive).
   * @param profile [out] resulting profile, untouched if the name is unknown.
   * @return true if @p autopilot_name is a known autopilot family.
   */
  static bool create(const std::string & autopilot_name, AutopilotProfile & profile);
};

/**
 * @brief Parse a vehicle type name.
 *
 * @param vehicle_name "copter" / "multirotor" (case insensitive).
 * @param vehicle_type [out] resulting vehicle type, untouched if unknown.
 * @return true if @p vehicle_name is a supported vehicle type.
 */
bool parseVehicleType(const std::string & vehicle_name, VehicleType & vehicle_type);

/**
 * @brief Parse the AttitudeTarget thrust semantics name.
 *
 * @param semantics_name "climb_rate" or "normalized_thrust" (case insensitive).
 * @param semantics [out] resulting semantics, untouched if unknown.
 * @return true if @p semantics_name is a known value.
 */
bool parseThrustSemantics(const std::string & semantics_name, ThrustSemantics & semantics);

}  // namespace as2_platform_mavlink

#endif  // AS2_PLATFORM_MAVLINK__AUTOPILOT_PROFILE_HPP_