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
 * @file autopilot_profile.cpp
 *
 * Autopilot-dependent behaviour of the MAVLink platform.
 *
 * @author Mohamed Elmahlawy
 */

#include "as2_platform_mavlink/autopilot_profile.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace as2_platform_mavlink
{

namespace
{

std::string toLower(const std::string & text)
{
  std::string lowered = text;
  std::transform(
    lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char character) {return std::tolower(character);});
  return lowered;
}

}  // namespace

bool AutopilotProfile::create(const std::string & autopilot_name, AutopilotProfile & profile)
{
  const std::string name = toLower(autopilot_name);

  if (name == "ardupilot" || name == "apm" || name == "arducopter") {
    // ArduPilot Copter, GUIDED mode.
    //  - Movement commands accepted in GUIDED: SET_POSITION_TARGET_LOCAL_NED
    //    (position and/or velocity and/or acceleration, plus yaw or yaw rate)
    //    and SET_ATTITUDE_TARGET.
    //  - Body rates of SET_ATTITUDE_TARGET are explicitly documented as
    //    "not supported", therefore ACRO control mode cannot be offered.
    //  - The thrust field of SET_ATTITUDE_TARGET is a climb rate unless
    //    GUID_OPTIONS bit 3 is set.
    //  - The vehicle will not leave the ground in GUIDED until it receives
    //    MAV_CMD_NAV_TAKEOFF.
    profile.family = AutopilotFamily::ARDUPILOT;
    profile.name = "ardupilot";
    profile.offboard_mode = "GUIDED";
    profile.manual_mode = "LOITER";
    profile.hold_mode = "BRAKE";
    profile.supports_acceleration_setpoints = true;
    profile.supports_body_rate_setpoints = false;
    profile.requires_takeoff_command = true;
    profile.set_mode_before_arming = true;
    profile.attitude_thrust = ThrustSemantics::CLIMB_RATE;
    return true;
  }

  if (name == "px4") {
    profile.family = AutopilotFamily::PX4;
    profile.name = "px4";
    profile.offboard_mode = "OFFBOARD";
    profile.manual_mode = "POSCTL";
    profile.hold_mode = "AUTO.LOITER";
    profile.supports_acceleration_setpoints = true;
    profile.supports_body_rate_setpoints = true;
    profile.requires_takeoff_command = false;
    profile.set_mode_before_arming = false;
    profile.attitude_thrust = ThrustSemantics::NORMALIZED_THRUST;
    return true;
  }

  return false;
}

bool parseVehicleType(const std::string & vehicle_name, VehicleType & vehicle_type)
{
  const std::string name = toLower(vehicle_name);
  if (name == "copter" || name == "multirotor" || name == "quadrotor") {
    vehicle_type = VehicleType::COPTER;
    return true;
  }
  return false;
}

bool parseThrustSemantics(const std::string & semantics_name, ThrustSemantics & semantics)
{
  const std::string name = toLower(semantics_name);
  if (name == "climb_rate") {
    semantics = ThrustSemantics::CLIMB_RATE;
    return true;
  }
  if (name == "normalized_thrust" || name == "thrust") {
    semantics = ThrustSemantics::NORMALIZED_THRUST;
    return true;
  }
  return false;
}

}  // namespace as2_platform_mavlink