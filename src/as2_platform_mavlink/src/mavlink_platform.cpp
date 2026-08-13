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
 * @file mavlink_platform.cpp
 *
 * MavlinkPlatform class implementation
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 *         Mohamed Elmahlawy (ArduPilot support)
 */

#include "as2_platform_mavlink/mavlink_platform.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "as2_core/utils/frame_utils.hpp"

namespace as2_platform_mavlink
{

namespace
{

/// MAV_CMD_COMPONENT_ARM_DISARM.
constexpr uint16_t kMavCmdComponentArmDisarm = 400;
/// Magic value of MAV_CMD_COMPONENT_ARM_DISARM param2 that forces the action.
constexpr float kForceArmDisarmMagic = 21196.0f;

/// Ignore flags of mavros_msgs::msg::PositionTarget, grouped by field.
constexpr uint16_t kIgnorePosition = mavros_msgs::msg::PositionTarget::IGNORE_PX |
  mavros_msgs::msg::PositionTarget::IGNORE_PY |
  mavros_msgs::msg::PositionTarget::IGNORE_PZ;
constexpr uint16_t kIgnoreVelocity = mavros_msgs::msg::PositionTarget::IGNORE_VX |
  mavros_msgs::msg::PositionTarget::IGNORE_VY |
  mavros_msgs::msg::PositionTarget::IGNORE_VZ;
constexpr uint16_t kIgnoreAcceleration = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
  mavros_msgs::msg::PositionTarget::IGNORE_AFY |
  mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
constexpr uint16_t kIgnoreYaw = mavros_msgs::msg::PositionTarget::IGNORE_YAW;
constexpr uint16_t kIgnoreYawRate = mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

double interpolateAngle(double from, double to, double alpha)
{
  const double sin_value = (1.0 - alpha) * std::sin(from) + alpha * std::sin(to);
  const double cos_value = (1.0 - alpha) * std::cos(from) + alpha * std::cos(to);
  if (std::abs(sin_value) < 1e-9 && std::abs(cos_value) < 1e-9) {
    return from;
  }
  return std::atan2(sin_value, cos_value);
}

}  // namespace

MavlinkPlatform::MavlinkPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  configureSensors();

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");

  declareParameters();
  setupCommunications();

  // The base class optimistically assumes the platform is connected. The
  // MAVROS heartbeat is the only reliable source of truth, so start
  // disconnected and let mavlinkStateCb() report the link status.
  this->platform_info_msg_.set__connected(false);

  RCLCPP_INFO(
    this->get_logger(), "MAVLink platform configured for autopilot '%s' (vehicle: copter)",
    autopilot_.name.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Flight modes: offboard '%s', manual '%s', hold '%s'",
    autopilot_.offboard_mode.c_str(), autopilot_.manual_mode.c_str(),
    autopilot_.hold_mode.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Simulation mode: %s",
    this->get_parameter("use_sim_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "External odometry mode: %s", external_odom_ ? "true" : "false");
}

/* ------------------------------------------------------------------------ */
/* Initialization                                                            */
/* ------------------------------------------------------------------------ */

void MavlinkPlatform::declareParameters()
{
  // Autopilot family. Everything firmware specific is derived from it.
  const std::string autopilot_name =
    this->declare_parameter<std::string>("autopilot", "ardupilot");
  if (!AutopilotProfile::create(autopilot_name, autopilot_)) {
    RCLCPP_FATAL(
      this->get_logger(), "Unknown autopilot '%s'. Supported values: 'ardupilot', 'px4'.",
      autopilot_name.c_str());
    throw std::runtime_error("Unknown autopilot: " + autopilot_name);
  }

  const std::string vehicle_name = this->declare_parameter<std::string>("vehicle_type", "copter");
  if (!parseVehicleType(vehicle_name, vehicle_type_)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Unsupported vehicle type '%s'. Only multirotors ('copter') are implemented: fixed wing "
      "and VTOL vehicles need different takeoff, landing and GUIDED semantics.",
      vehicle_name.c_str());
    throw std::runtime_error("Unsupported vehicle type: " + vehicle_name);
  }

  // Flight mode names default to the autopilot profile and may be overridden,
  // e.g. to use "GUIDED_NOGPS" or "ALT_HOLD".
  autopilot_.offboard_mode =
    this->declare_parameter<std::string>("modes.offboard", autopilot_.offboard_mode);
  autopilot_.manual_mode =
    this->declare_parameter<std::string>("modes.manual", autopilot_.manual_mode);
  autopilot_.hold_mode = this->declare_parameter<std::string>("modes.hold", autopilot_.hold_mode);

  // ArduPilot Copter interprets the SET_ATTITUDE_TARGET thrust field as a
  // climb rate unless GUID_OPTIONS bit 3 is set.
  const std::string thrust_semantics = this->declare_parameter<std::string>(
    "attitude_thrust_semantics",
    autopilot_.attitude_thrust == ThrustSemantics::CLIMB_RATE ? "climb_rate" : "normalized_thrust");
  if (!parseThrustSemantics(thrust_semantics, autopilot_.attitude_thrust)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Unknown attitude_thrust_semantics '%s'. Supported: 'climb_rate', 'normalized_thrust'.",
      thrust_semantics.c_str());
    throw std::runtime_error("Unknown attitude_thrust_semantics: " + thrust_semantics);
  }

  // Thrust limits are only used by the ATTITUDE control mode.
  max_thrust_ = static_cast<float>(this->declare_parameter<double>("max_thrust", 0.0));
  min_thrust_ = static_cast<float>(this->declare_parameter<double>("min_thrust", 0.0));

  external_odom_ = this->declare_parameter<bool>("external_odom", false);

  // Arming as a side effect of a mode change is dangerous on a real airframe,
  // so it is opt-in (it was unconditional in the PX4 only implementation).
  arm_on_offboard_ = this->declare_parameter<bool>("arm_on_offboard", false);

  command_timeout_ = this->declare_parameter<double>("control.command_timeout", 0.5);
  connection_timeout_ = this->declare_parameter<double>("connection.timeout", 2.0);
  mode_change_timeout_ = this->declare_parameter<double>("control.mode_change_timeout", 5.0);

  takeoff_height_ = this->declare_parameter<double>("takeoff.height", 1.0);
  takeoff_height_tolerance_ = this->declare_parameter<double>("takeoff.height_tolerance", 0.15);
  takeoff_timeout_ = this->declare_parameter<double>("takeoff.timeout", 30.0);
  land_timeout_ = this->declare_parameter<double>("land.timeout", 60.0);
  blocking_takeoff_and_land_ = this->declare_parameter<bool>("takeoff.blocking", true);

  send_acceleration_ = this->declare_parameter<bool>("trajectory.send_acceleration", true);
  if (send_acceleration_ && !autopilot_.supports_acceleration_setpoints) {
    RCLCPP_WARN(
      this->get_logger(),
      "Autopilot '%s' does not honour acceleration setpoints, disabling the acceleration "
      "feed-forward.", autopilot_.name.c_str());
    send_acceleration_ = false;
  }
  trajectory_sampling_dt_ = this->declare_parameter<double>("trajectory.sampling_dt", 0.01);
  if (trajectory_sampling_dt_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "trajectory.sampling_dt must be positive, got %f. Falling back to 0.01 s.",
      trajectory_sampling_dt_);
    trajectory_sampling_dt_ = 0.01;
  }
}

void MavlinkPlatform::setupCommunications()
{
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  // All autopilot feedback lives in a dedicated callback group so that it can
  // keep flowing while a blocking platform service call is in progress.
  feedback_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  feedback_executor_.add_callback_group(
    feedback_callback_group_, this->get_node_base_interface());

  rclcpp::SubscriptionOptions feedback_options;
  feedback_options.callback_group = feedback_callback_group_;

  mavlink_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "mavros/state", rclcpp::SensorDataQoS(),
    std::bind(&MavlinkPlatform::mavlinkStateCb, this, std::placeholders::_1), feedback_options);

  mavlink_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "mavros/local_position/odom", rclcpp::SensorDataQoS(),
    std::bind(&MavlinkPlatform::mavlinkOdomCb, this, std::placeholders::_1), feedback_options);

  mavlink_extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "mavros/extended_state", rclcpp::SensorDataQoS(),
    std::bind(&MavlinkPlatform::mavlinkExtendedStateCb, this, std::placeholders::_1),
    feedback_options);

  // The feedback executor is pumped from the node executor during normal
  // operation and from waitFor() while a blocking command is running.
  feedback_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / std::max(cmd_freq_, 1.0f)),
    std::bind(&MavlinkPlatform::pumpFeedback, this));

  mavlink_arm_client_ =
    std::make_shared<as2::SynchronousServiceClient<mavros_msgs::srv::CommandBool>>(
    "mavros/cmd/arming", this);
  mavlink_set_mode_client_ =
    std::make_shared<as2::SynchronousServiceClient<mavros_msgs::srv::SetMode>>(
    "mavros/set_mode", this);
  mavlink_command_long_client_ =
    std::make_shared<as2::SynchronousServiceClient<mavros_msgs::srv::CommandLong>>(
    "mavros/cmd/command", this);
  mavlink_takeoff_client_ =
    std::make_shared<as2::SynchronousServiceClient<mavros_msgs::srv::CommandTOL>>(
    "mavros/cmd/takeoff", this);
  mavlink_land_client_ =
    std::make_shared<as2::SynchronousServiceClient<mavros_msgs::srv::CommandTOL>>(
    "mavros/cmd/land", this);

  // Every position, velocity, acceleration and trajectory setpoint goes
  // through setpoint_raw/local, which maps one to one to
  // SET_POSITION_TARGET_LOCAL_NED and is the only MAVROS interface exposing
  // the type_mask and the coordinate frame.
  mavlink_setpoint_raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "mavros/setpoint_raw/local", rclcpp::SensorDataQoS());
  mavlink_attitude_setpoint_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
    "mavros/setpoint_raw/attitude", rclcpp::SensorDataQoS());

  if (external_odom_) {
    mavlink_vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/vision_pose/pose", rclcpp::SensorDataQoS());
    mavlink_vision_speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "mavros/vision_speed/speed_twist", rclcpp::SensorDataQoS());

    external_odometry_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_global_name(as2_names::topics::self_localization::twist),
      as2_names::topics::self_localization::qos,
      std::bind(&MavlinkPlatform::externalOdomCb, this, std::placeholders::_1));

    vision_pose_timer_ = this->create_timer(
      std::chrono::milliseconds(10), std::bind(&MavlinkPlatform::publishVisualOdometry, this));
  }
}

void MavlinkPlatform::configureSensors()
{
  // Only the odometry is republished by the platform: the IMU, GPS and
  // battery measurements are remapped by the MAVROS launch file straight into
  // the Aerostack2 sensor_measurements namespace.
  odometry_raw_estimation_ptr_ =
    std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
}

/* ------------------------------------------------------------------------ */
/* Feedback                                                                  */
/* ------------------------------------------------------------------------ */

void MavlinkPlatform::pumpFeedback()
{
  feedback_executor_.spin_some(std::chrono::milliseconds(0));

  if (!state_received_) {
    return;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_state_time_).count();
  if (elapsed > connection_timeout_ && this->getConnectedStatus()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No MAVROS state received for %.1f s, marking the platform as disconnected", elapsed);
    this->platform_info_msg_.set__connected(false);
    this->platform_info_msg_.set__offboard(false);
  }
}

void MavlinkPlatform::mavlinkStateCb(const mavros_msgs::msg::State::SharedPtr msg)
{
  state_received_ = true;
  last_state_time_ = std::chrono::steady_clock::now();

  if (msg->connected && !this->getConnectedStatus()) {
    RCLCPP_INFO(this->get_logger(), "Autopilot link established");
  }
  this->platform_info_msg_.set__connected(msg->connected);
  this->platform_info_msg_.set__armed(msg->armed);

  if (msg->mode != autopilot_mode_) {
    RCLCPP_INFO(
      this->get_logger(), "Autopilot flight mode changed: '%s' -> '%s'", autopilot_mode_.c_str(),
      msg->mode.c_str());
    autopilot_mode_ = msg->mode;
    // A mode change commanded from the RC or the GCS invalidates a pending
    // hold request issued by ownStopPlatform().
    stop_requested_ = false;
  }

  // Aerostack2 "offboard" means "the autopilot accepts onboard setpoints",
  // which is GUIDED for ArduPilot and OFFBOARD for PX4.
  this->platform_info_msg_.set__offboard(msg->mode == autopilot_.offboard_mode);
}

void MavlinkPlatform::mavlinkOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // mavros/local_position/odom is expressed in the ENU frame anchored at the
  // autopilot EKF origin, which is what Aerostack2 calls <namespace>/odom.
  msg->header.frame_id = odom_frame_id_;
  msg->child_frame_id = base_link_frame_id_;
  odometry_raw_estimation_ptr_->updateData(*msg);

  last_odom_pose_.header = msg->header;
  last_odom_pose_.pose = msg->pose.pose;
  odometry_received_ = true;
}

void MavlinkPlatform::mavlinkExtendedStateCb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
  landed_state_ = msg->landed_state;
}

void MavlinkPlatform::externalOdomCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  try {
    auto [pose_msg, twist_msg] = tf_handler_->getState(
      *msg, base_link_frame_id_, odom_frame_id_, base_link_frame_id_);

    mavlink_vision_speed_msg_.header = twist_msg.header;    // BODY_FRAME_FLU
    mavlink_vision_speed_msg_.twist = twist_msg.twist;

    mavlink_vision_pose_msg_.header = pose_msg.header;      // LOCAL_FRAME_FLU
    mavlink_vision_pose_msg_.pose = pose_msg.pose;
  } catch (const tf2::TransformException & ex) {
    auto & clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), clk, 1000, "Could not get external odometry transform: %s", ex.what());
  }
}

void MavlinkPlatform::publishVisualOdometry()
{
  mavlink_vision_pose_pub_->publish(mavlink_vision_pose_msg_);
  mavlink_vision_speed_pub_->publish(mavlink_vision_speed_msg_);
}

bool MavlinkPlatform::waitFor(
  const std::function<bool()> & condition, double timeout, const std::string & description)
{
  // The steady clock is used on purpose: this loop must terminate even when
  // use_sim_time is enabled and the /clock topic stalls.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout));

  while (rclcpp::ok()) {
    feedback_executor_.spin_some(std::chrono::milliseconds(10));
    if (condition()) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        this->get_logger(), "Timeout (%.1f s) while waiting for %s", timeout, description.c_str());
      return false;
    }
  }
  return false;
}

/* ------------------------------------------------------------------------ */
/* Autopilot commands                                                        */
/* ------------------------------------------------------------------------ */

bool MavlinkPlatform::setFlightMode(const std::string & flight_mode, bool confirm)
{
  if (flight_mode.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Empty flight mode requested");
    return false;
  }
  if (autopilot_mode_ == flight_mode) {
    return true;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  auto response = std::make_shared<mavros_msgs::srv::SetMode::Response>();
  request->custom_mode = flight_mode;

  if (!mavlink_set_mode_client_->sendRequest(request, response, 3)) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to call mavros/set_mode for mode '%s'", flight_mode.c_str());
    return false;
  }
  if (!response->mode_sent) {
    RCLCPP_ERROR(this->get_logger(), "MAVROS rejected flight mode '%s'", flight_mode.c_str());
    return false;
  }
  if (!confirm) {
    return true;
  }

  // mode_sent only means the message left MAVROS: the autopilot may still
  // refuse the mode, e.g. GUIDED without a valid position estimate.
  return waitFor(
    [this, &flight_mode]() {return autopilot_mode_ == flight_mode;}, mode_change_timeout_,
    "flight mode '" + flight_mode + "'");
}

bool MavlinkPlatform::sendCommandLong(
  uint16_t command, float param1, float param2, float param3, float param4, float param5,
  float param6, float param7)
{
  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  auto response = std::make_shared<mavros_msgs::srv::CommandLong::Response>();
  request->command = command;
  request->broadcast = false;
  request->confirmation = 0;
  request->param1 = param1;
  request->param2 = param2;
  request->param3 = param3;
  request->param4 = param4;
  request->param5 = param5;
  request->param6 = param6;
  request->param7 = param7;

  if (!mavlink_command_long_client_->sendRequest(request, response, 3)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call mavros/cmd/command (MAV_CMD %u)", command);
    return false;
  }
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Autopilot rejected MAV_CMD %u (MAV_RESULT %u)", command,
      response->result);
    return false;
  }
  return true;
}

bool MavlinkPlatform::ownSetArmingState(bool state)
{
  if (state) {
    if (!this->getConnectedStatus()) {
      RCLCPP_ERROR(this->get_logger(), "Refusing to arm: no autopilot connection");
      return false;
    }
    if (!odometry_received_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Refusing to arm: no local position received yet. The autopilot EKF has no origin, so "
        "GUIDED setpoints would be rejected. Check the GPS fix and the EKF status.");
      return false;
    }
    // ArduPilot runs its arming checks against the *current* flight mode:
    // entering GUIDED first makes the arming result meaningful and avoids the
    // vehicle sitting armed in a mode that ignores onboard setpoints.
    if (autopilot_.set_mode_before_arming && autopilot_mode_ != autopilot_.offboard_mode) {
      RCLCPP_INFO(
        this->get_logger(), "Switching to '%s' before arming", autopilot_.offboard_mode.c_str());
      if (!setFlightMode(autopilot_.offboard_mode, true)) {
        return false;
      }
    }
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  auto response = std::make_shared<mavros_msgs::srv::CommandBool::Response>();
  request->value = state;

  if (!mavlink_arm_client_->sendRequest(request, response, 3)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call mavros/cmd/arming");
    return false;
  }
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Autopilot refused to %s (MAV_RESULT %u). Check the pre-arm checks.",
      state ? "arm" : "disarm", response->result);
    return false;
  }
  return true;
}

bool MavlinkPlatform::ownSetOffboardControl(bool offboard)
{
  if (!offboard) {
    if (autopilot_.manual_mode.empty()) {
      RCLCPP_ERROR(
        this->get_logger(), "Leaving offboard control is disabled (modes.manual is empty)");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "Leaving offboard control, switching to '%s'",
      autopilot_.manual_mode.c_str());
    return setFlightMode(autopilot_.manual_mode, true);
  }

  if (!this->getConnectedStatus()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot enter offboard control: no autopilot connection");
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "Switching to offboard control mode '%s'",
    autopilot_.offboard_mode.c_str());
  if (!setFlightMode(autopilot_.offboard_mode, true)) {
    return false;
  }

  if (arm_on_offboard_ && !this->getArmingState()) {
    RCLCPP_INFO(this->get_logger(), "arm_on_offboard is enabled, arming the vehicle");
    if (!this->ownSetArmingState(true)) {
      return false;
    }
  }
  return true;
}

bool MavlinkPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  has_mode_settled_ = false;
  hover_setpoint_latched_ = false;

  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
      RCLCPP_INFO(this->get_logger(), "Control mode UNSET");
      break;
    case as2_msgs::msg::ControlMode::HOVER: {
        if (!odometry_received_) {
          RCLCPP_ERROR(this->get_logger(), "Cannot hover: no local position available");
          return false;
        }
        // Hover is a position hold at the current pose, which keeps the
        // vehicle in the offboard flight mode and Aerostack2 in control.
        hover_pose_ = last_odom_pose_;
        hover_setpoint_latched_ = true;
        RCLCPP_INFO(
          this->get_logger(), "HOVER enabled, holding [%.2f, %.2f, %.2f] in frame '%s'",
          hover_pose_.pose.position.x, hover_pose_.pose.position.y, hover_pose_.pose.position.z,
          hover_pose_.header.frame_id.c_str());
      } break;
    case as2_msgs::msg::ControlMode::POSITION:
      RCLCPP_INFO(this->get_logger(), "POSITION mode enabled");
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      RCLCPP_INFO(this->get_logger(), "SPEED mode enabled");
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      RCLCPP_INFO(
        this->get_logger(), "TRAJECTORY mode enabled (acceleration feed-forward: %s)",
        send_acceleration_ ? "on" : "off");
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE: {
        if (autopilot_.attitude_thrust == ThrustSemantics::CLIMB_RATE) {
          RCLCPP_ERROR(
            this->get_logger(),
            "ATTITUDE mode is not available: this autopilot interprets the SET_ATTITUDE_TARGET "
            "thrust field as a climb rate. Set the autopilot parameter GUID_OPTIONS bit 3 and the "
            "platform parameter attitude_thrust_semantics to 'normalized_thrust' to enable it.");
          return false;
        }
        if (max_thrust_ <= 0.0f) {
          RCLCPP_ERROR(this->get_logger(), "ATTITUDE mode requires a positive max_thrust");
          return false;
        }
        RCLCPP_INFO(this->get_logger(), "ATTITUDE mode enabled");
      } break;
    case as2_msgs::msg::ControlMode::ACRO:
      RCLCPP_ERROR(
        this->get_logger(),
        "ACRO mode is not supported by autopilot '%s': the body rate fields of "
        "SET_ATTITUDE_TARGET are ignored by the firmware.", autopilot_.name.c_str());
      return false;
    default:
      RCLCPP_WARN(this->get_logger(), "Control mode %d is not supported", msg.control_mode);
      return false;
  }

  has_mode_settled_ = true;
  return true;
}

/* ------------------------------------------------------------------------ */
/* Takeoff and landing                                                       */
/* ------------------------------------------------------------------------ */

bool MavlinkPlatform::ownTakeoff()
{
  if (!autopilot_.requires_takeoff_command) {
    RCLCPP_WARN(
      this->get_logger(),
      "Autopilot '%s' does not implement an autopilot side takeoff, use a motion behavior plugin "
      "instead of 'takeoff_plugin_platform'.", autopilot_.name.c_str());
    return false;
  }
  if (!this->getConnectedStatus() || !odometry_received_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot take off: the platform is not ready");
    return false;
  }
  if (!setFlightMode(autopilot_.offboard_mode, true)) {
    RCLCPP_ERROR(
      this->get_logger(), "Cannot take off: '%s' mode could not be entered",
      autopilot_.offboard_mode.c_str());
    return false;
  }
  if (!this->getArmingState()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot take off: the vehicle is not armed");
    return false;
  }

  const double target_height = last_odom_pose_.pose.position.z + takeoff_height_;
  RCLCPP_INFO(this->get_logger(), "Taking off %.2f m (MAV_CMD_NAV_TAKEOFF)", takeoff_height_);

  // mavros/cmd/takeoff maps to MAV_CMD_NAV_TAKEOFF. ArduPilot Copter only
  // uses param7 (altitude), which is relative to home.
  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  auto response = std::make_shared<mavros_msgs::srv::CommandTOL::Response>();
  request->min_pitch = 0.0f;
  request->yaw = 0.0f;
  request->latitude = 0.0f;
  request->longitude = 0.0f;
  request->altitude = static_cast<float>(takeoff_height_);

  if (!mavlink_takeoff_client_->sendRequest(request, response, 3)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call mavros/cmd/takeoff");
    return false;
  }
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Autopilot rejected the takeoff command (MAV_RESULT %u)",
      response->result);
    return false;
  }

  if (!blocking_takeoff_and_land_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Non blocking takeoff: the takeoff behavior reports success before the target height is "
      "reached.");
    return true;
  }

  const bool reached = waitFor(
    [this, target_height]() {
      return last_odom_pose_.pose.position.z >= target_height - takeoff_height_tolerance_;
    },
    takeoff_timeout_, "the takeoff to complete");

  if (!reached) {
    RCLCPP_ERROR(
      this->get_logger(), "Takeoff did not reach %.2f m (current height %.2f m)", target_height,
      last_odom_pose_.pose.position.z);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Takeoff completed at %.2f m", last_odom_pose_.pose.position.z);
  return true;
}

bool MavlinkPlatform::ownLand()
{
  RCLCPP_INFO(this->get_logger(), "Landing (MAV_CMD_NAV_LAND)");

  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  auto response = std::make_shared<mavros_msgs::srv::CommandTOL::Response>();

  if (!mavlink_land_client_->sendRequest(request, response, 3)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call mavros/cmd/land");
    return false;
  }
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Autopilot rejected the land command (MAV_RESULT %u)", response->result);
    return false;
  }

  // The autopilot leaves the offboard flight mode while landing, so Aerostack2
  // must stop streaming setpoints immediately.
  this->platform_info_msg_.set__offboard(false);

  if (!blocking_takeoff_and_land_) {
    return true;
  }

  const bool landed = waitFor(
    [this]() {
      return landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND ||
      !this->getArmingState();
    },
    land_timeout_, "the landing to complete");

  if (!landed) {
    RCLCPP_ERROR(this->get_logger(), "Landing did not complete within the timeout");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Landing completed");
  return true;
}

void MavlinkPlatform::ownKillSwitch()
{
  RCLCPP_ERROR(this->get_logger(), "KILL SWITCH TRIGGERED");
  if (!sendCommandLong(kMavCmdComponentArmDisarm, 0.0f, kForceArmDisarmMagic)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to force disarm the vehicle");
  }
}

void MavlinkPlatform::ownStopPlatform()
{
  // This is called on every command cycle while the platform is in EMERGENCY,
  // so the mode change must only be requested once.
  if (stop_requested_) {
    return;
  }
  stop_requested_ = true;
  RCLCPP_WARN(
    this->get_logger(), "Emergency hover requested, switching to '%s'",
    autopilot_.hold_mode.c_str());
  if (!setFlightMode(autopilot_.hold_mode, false)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not switch to '%s'. The vehicle will brake by itself once the setpoint stream stops.",
      autopilot_.hold_mode.c_str());
  }
}

/* ------------------------------------------------------------------------ */
/* Setpoint generation                                                       */
/* ------------------------------------------------------------------------ */

bool MavlinkPlatform::isReferenceFresh(
  const builtin_interfaces::msg::Time & stamp, const char * reference_name)
{
  auto & clk = *this->get_clock();
  const rclcpp::Time reference_time(stamp, this->get_clock()->get_clock_type());
  if (reference_time.nanoseconds() == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), clk, 2000, "No %s reference received yet", reference_name);
    return false;
  }
  const double age = (this->now() - reference_time).seconds();
  if (age > command_timeout_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), clk, 1000,
      "The %s reference is %.2f s old (timeout %.2f s), stopping the setpoint stream. The "
      "autopilot will brake and hold position.", reference_name, age, command_timeout_);
    return false;
  }
  return true;
}

void MavlinkPlatform::publishPositionTarget(const mavros_msgs::msg::PositionTarget & msg)
{
  mavlink_setpoint_raw_pub_->publish(msg);
}

bool MavlinkPlatform::sendHoverSetpoint()
{
  if (!hover_setpoint_latched_) {
    return false;
  }
  mavros_msgs::msg::PositionTarget target;
  target.header.stamp = this->now();
  target.header.frame_id = odom_frame_id_;
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = kIgnoreVelocity | kIgnoreAcceleration | kIgnoreYawRate;
  target.position = hover_pose_.pose.position;
  target.yaw = static_cast<float>(as2::frame::getYawFromQuaternion(hover_pose_.pose.orientation));
  publishPositionTarget(target);
  return true;
}

bool MavlinkPlatform::sendPositionSetpoint(const as2_msgs::msg::ControlMode & mode)
{
  if (!isReferenceFresh(this->command_pose_msg_.header.stamp, "position")) {
    return false;
  }

  geometry_msgs::msg::PoseStamped pose = this->command_pose_msg_;
  if (pose.header.frame_id != odom_frame_id_ && !tf_handler_->tryConvert(pose, odom_frame_id_)) {
    auto & clk = *this->get_clock();
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), clk, 1000, "Cannot convert the position reference from '%s' to '%s'",
      pose.header.frame_id.c_str(), odom_frame_id_.c_str());
    return false;
  }

  mavros_msgs::msg::PositionTarget target;
  target.header.stamp = this->now();
  target.header.frame_id = odom_frame_id_;
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = kIgnoreVelocity | kIgnoreAcceleration;
  target.position = pose.pose.position;

  if (mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    target.type_mask |= kIgnoreYaw;
    // In POSITION + YAW_SPEED the twist reference carries the desired yaw rate.
    target.yaw_rate = static_cast<float>(this->command_twist_msg_.twist.angular.z);
  } else {
    target.type_mask |= kIgnoreYawRate;
    target.yaw = static_cast<float>(as2::frame::getYawFromQuaternion(pose.pose.orientation));
  }

  publishPositionTarget(target);
  return true;
}

bool MavlinkPlatform::sendSpeedSetpoint(const as2_msgs::msg::ControlMode & mode)
{
  if (!isReferenceFresh(this->command_twist_msg_.header.stamp, "speed")) {
    return false;
  }

  geometry_msgs::msg::TwistStamped twist = this->command_twist_msg_;
  if (twist.header.frame_id != odom_frame_id_ && !tf_handler_->tryConvert(twist, odom_frame_id_)) {
    auto & clk = *this->get_clock();
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), clk, 1000, "Cannot convert the speed reference from '%s' to '%s'",
      twist.header.frame_id.c_str(), odom_frame_id_.c_str());
    return false;
  }

  mavros_msgs::msg::PositionTarget target;
  target.header.stamp = this->now();
  target.header.frame_id = odom_frame_id_;
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = kIgnorePosition | kIgnoreAcceleration;
  target.velocity.x = twist.twist.linear.x;
  target.velocity.y = twist.twist.linear.y;
  target.velocity.z = twist.twist.linear.z;

  if (mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
    target.type_mask |= kIgnoreYawRate;
    target.yaw = static_cast<float>(
      as2::frame::getYawFromQuaternion(this->command_pose_msg_.pose.orientation));
  } else {
    target.type_mask |= kIgnoreYaw;
    target.yaw_rate = static_cast<float>(twist.twist.angular.z);
  }

  publishPositionTarget(target);
  return true;
}

bool MavlinkPlatform::sampleTrajectory(as2_msgs::msg::TrajectoryPoint & setpoint) const
{
  const auto & setpoints = this->command_trajectory_msg_.setpoints;
  if (setpoints.empty()) {
    return false;
  }
  if (setpoints.size() == 1) {
    setpoint = setpoints.front();
    return true;
  }

  const rclcpp::Time reference_time(
    this->command_trajectory_msg_.header.stamp, this->get_clock()->get_clock_type());
  const double elapsed = (this->now() - reference_time).seconds();
  const double horizon = static_cast<double>(setpoints.size() - 1) * trajectory_sampling_dt_;
  const double time_in_horizon = std::clamp(elapsed, 0.0, horizon);

  const double index_real = time_in_horizon / trajectory_sampling_dt_;
  const size_t index = std::min(static_cast<size_t>(index_real), setpoints.size() - 2);
  const double alpha = std::clamp(index_real - static_cast<double>(index), 0.0, 1.0);

  const auto & first = setpoints[index];
  const auto & second = setpoints[index + 1];
  const auto lerp = [alpha](double a, double b) {return a + alpha * (b - a);};

  setpoint.position.x = lerp(first.position.x, second.position.x);
  setpoint.position.y = lerp(first.position.y, second.position.y);
  setpoint.position.z = lerp(first.position.z, second.position.z);
  setpoint.twist.x = lerp(first.twist.x, second.twist.x);
  setpoint.twist.y = lerp(first.twist.y, second.twist.y);
  setpoint.twist.z = lerp(first.twist.z, second.twist.z);
  setpoint.acceleration.x = lerp(first.acceleration.x, second.acceleration.x);
  setpoint.acceleration.y = lerp(first.acceleration.y, second.acceleration.y);
  setpoint.acceleration.z = lerp(first.acceleration.z, second.acceleration.z);
  setpoint.yaw_angle =
    static_cast<float>(interpolateAngle(first.yaw_angle, second.yaw_angle, alpha));
  return true;
}

bool MavlinkPlatform::sendTrajectorySetpoint(const as2_msgs::msg::ControlMode & mode)
{
  if (!isReferenceFresh(this->command_trajectory_msg_.header.stamp, "trajectory")) {
    return false;
  }
  // The controller publishes trajectory references in the LOCAL_ENU frame,
  // which is already the MAVROS local ENU frame. The conversion below is a
  // safety net for references produced in any other frame.
  if (this->command_trajectory_msg_.header.frame_id != odom_frame_id_) {
    if (!tf_handler_->tryConvert(this->command_trajectory_msg_, odom_frame_id_)) {
      auto & clk = *this->get_clock();
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), clk, 1000,
        "Cannot convert the trajectory reference from '%s' to '%s'",
        this->command_trajectory_msg_.header.frame_id.c_str(), odom_frame_id_.c_str());
      return false;
    }
  }

  as2_msgs::msg::TrajectoryPoint setpoint;
  if (!sampleTrajectory(setpoint)) {
    auto & clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "Empty trajectory reference received");
    return false;
  }

  // SET_POSITION_TARGET_LOCAL_NED with position, velocity and acceleration
  // enabled: ArduPilot tracks the position while using the velocity and the
  // acceleration as feed-forward terms. This is what keeps a sampled
  // trajectory smooth instead of degrading it into independent waypoints.
  mavros_msgs::msg::PositionTarget target;
  target.header.stamp = this->now();
  target.header.frame_id = odom_frame_id_;
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = 0;
  if (!send_acceleration_) {
    target.type_mask |= kIgnoreAcceleration;
  }

  target.position.x = setpoint.position.x;
  target.position.y = setpoint.position.y;
  target.position.z = setpoint.position.z;
  target.velocity = setpoint.twist;
  target.acceleration_or_force = setpoint.acceleration;

  if (mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    target.type_mask |= kIgnoreYaw;
    target.yaw_rate = static_cast<float>(this->command_twist_msg_.twist.angular.z);
  } else {
    target.type_mask |= kIgnoreYawRate;
    target.yaw = setpoint.yaw_angle;
  }

  publishPositionTarget(target);
  return true;
}

bool MavlinkPlatform::sendAttitudeSetpoint()
{
  if (!isReferenceFresh(this->command_pose_msg_.header.stamp, "attitude")) {
    return false;
  }

  float thrust = this->command_thrust_msg_.thrust / max_thrust_;
  thrust = std::clamp(thrust, min_thrust_, 1.0f);

  mavros_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = odom_frame_id_;
  msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
    mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
    mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
  msg.orientation = this->command_pose_msg_.pose.orientation;
  msg.thrust = thrust;
  mavlink_attitude_setpoint_pub_->publish(msg);
  return true;
}

bool MavlinkPlatform::ownSendCommand()
{
  if (!has_mode_settled_) {
    return false;
  }

  const as2_msgs::msg::ControlMode & mode = this->getControlMode();
  switch (mode.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER:
      return sendHoverSetpoint();
    case as2_msgs::msg::ControlMode::POSITION:
      return sendPositionSetpoint(mode);
    case as2_msgs::msg::ControlMode::SPEED:
      return sendSpeedSetpoint(mode);
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      return sendTrajectorySetpoint(mode);
    case as2_msgs::msg::ControlMode::ATTITUDE:
      return sendAttitudeSetpoint();
    default: {
        auto & clk = *this->get_clock();
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), clk, 5000, "Control mode %d cannot be sent to the autopilot",
          mode.control_mode);
        return false;
      }
  }
}

}  // namespace as2_platform_mavlink