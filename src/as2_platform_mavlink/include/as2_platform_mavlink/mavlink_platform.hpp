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
 * @file mavlink_platform.hpp
 *
 * MavlinkPlatform class definition
 *
 * Aerostack2 aerial platform for autopilots reachable through MAVROS.
 * Autopilot specific behaviour is described by as2_platform_mavlink::AutopilotProfile.
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 *         Mohamed Elmahlawy (ArduPilot support)
 */

#ifndef AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_
#define AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <rclcpp/rclcpp.hpp>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"
#include "as2_platform_mavlink/autopilot_profile.hpp"

namespace as2_platform_mavlink
{

/**
 * @brief Aerostack2 aerial platform for MAVLink autopilots driven through MAVROS.
 *
 * Coordinate frames
 * -----------------
 * Every setpoint published by this platform is expressed in the MAVROS local
 * ENU frame, which is the frame of `mavros/local_position/odom` and is
 * re-labelled by this platform as `<namespace>/odom`. MAVROS converts ENU to
 * the NED frame expected by MAVLink internally (see the `setpoint_raw`
 * plugin), so the messages published here must *not* be pre-converted.
 * `MAV_FRAME_LOCAL_NED` targets are relative to the autopilot EKF origin,
 * which is precisely the origin of `mavros/local_position/odom`.
 */
class MavlinkPlatform : public as2::AerialPlatform
{
public:
  explicit MavlinkPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MavlinkPlatform() {}

  void configureSensors() override;

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  bool ownSendCommand() override;
  bool ownTakeoff() override;
  bool ownLand() override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

private:
  /* -------------------------------------------------------------------- */
  /* Initialization                                                        */
  /* -------------------------------------------------------------------- */

  void declareParameters();
  void setupCommunications();

  /* -------------------------------------------------------------------- */
  /* Feedback                                                              */
  /* -------------------------------------------------------------------- */

  /**
   * @brief Process the autopilot feedback that is pending in the feedback
   * executor.
   *
   * All MAVROS feedback subscriptions live in a dedicated callback group that
   * is *not* served by the node executor, so that blocking operations
   * (takeoff, landing, flight mode confirmation) can keep consuming autopilot
   * feedback while the node executor is busy inside a service callback.
   */
  void pumpFeedback();

  void mavlinkStateCb(const mavros_msgs::msg::State::SharedPtr msg);
  void mavlinkOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void mavlinkExtendedStateCb(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
  void externalOdomCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief Block until @p condition holds, while keeping the autopilot
   * feedback flowing.
   *
   * @param condition predicate evaluated after each feedback batch.
   * @param timeout maximum wall-clock time to wait.
   * @param description human readable description used for logging.
   * @return true if the condition became true before the timeout.
   */
  bool waitFor(
    const std::function<bool()> & condition, double timeout,
    const std::string & description);

  /* -------------------------------------------------------------------- */
  /* Autopilot commands                                                    */
  /* -------------------------------------------------------------------- */

  /**
   * @brief Request a flight mode change and optionally confirm it.
   *
   * @param flight_mode autopilot custom mode name, e.g. "GUIDED".
   * @param confirm wait until the autopilot reports the requested mode.
   * @return true if the mode was accepted (and confirmed, when requested).
   */
  bool setFlightMode(const std::string & flight_mode, bool confirm);

  bool sendCommandLong(
    uint16_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
    float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

  /* -------------------------------------------------------------------- */
  /* Setpoint generation                                                   */
  /* -------------------------------------------------------------------- */

  bool sendHoverSetpoint();
  bool sendPositionSetpoint(const as2_msgs::msg::ControlMode & mode);
  bool sendSpeedSetpoint(const as2_msgs::msg::ControlMode & mode);
  bool sendTrajectorySetpoint(const as2_msgs::msg::ControlMode & mode);
  bool sendAttitudeSetpoint();

  /**
   * @brief Sample the trajectory reference at the current time.
   *
   * The trajectory behavior emits a short horizon of `sampling_n` setpoints
   * spaced `trajectory_sampling_dt_` apart, starting at the message stamp.
   * The platform streams setpoints at `cmd_freq`, which is generally faster,
   * so the horizon is linearly interpolated in time. When a single setpoint
   * is published (the default) the interpolation degenerates to a hold.
   *
   * @param setpoint [out] interpolated setpoint.
   * @return true if a setpoint could be produced.
   */
  bool sampleTrajectory(as2_msgs::msg::TrajectoryPoint & setpoint) const;

  /**
   * @brief Check that a reference message is recent enough to be forwarded.
   *
   * ArduPilot stops the vehicle three seconds after the last velocity or
   * acceleration setpoint. Forwarding a stale reference for longer than
   * `command_timeout_` would keep the vehicle moving with an outdated
   * command, so streaming is stopped instead and the autopilot brakes.
   */
  bool isReferenceFresh(const builtin_interfaces::msg::Time & stamp, const char * reference_name);

  /// Publish a PositionTarget in the MAVROS local ENU frame.
  void publishPositionTarget(const mavros_msgs::msg::PositionTarget & msg);

  void publishVisualOdometry();

  /* -------------------------------------------------------------------- */
  /* Members                                                               */
  /* -------------------------------------------------------------------- */

  // Configuration
  AutopilotProfile autopilot_;
  VehicleType vehicle_type_ = VehicleType::COPTER;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  float max_thrust_ = 0.0f;
  float min_thrust_ = 0.0f;
  bool external_odom_ = false;
  bool arm_on_offboard_ = false;
  double command_timeout_ = 0.5;
  double connection_timeout_ = 2.0;
  double mode_change_timeout_ = 5.0;
  double takeoff_height_ = 1.0;
  double takeoff_height_tolerance_ = 0.15;
  double takeoff_timeout_ = 30.0;
  double land_timeout_ = 60.0;
  bool blocking_takeoff_and_land_ = true;
  bool send_acceleration_ = true;
  double trajectory_sampling_dt_ = 0.01;

  // Autopilot feedback
  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  rclcpp::executors::SingleThreadedExecutor feedback_executor_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavlink_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavlink_odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr mavlink_extended_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr external_odometry_sub_;

  std::string autopilot_mode_;
  bool odometry_received_ = false;
  geometry_msgs::msg::PoseStamped last_odom_pose_;
  uint8_t landed_state_ = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
  std::chrono::steady_clock::time_point last_state_time_ {};
  bool state_received_ = false;

  // Autopilot commands
  as2::SynchronousServiceClient<mavros_msgs::srv::CommandBool>::SharedPtr mavlink_arm_client_;
  as2::SynchronousServiceClient<mavros_msgs::srv::SetMode>::SharedPtr mavlink_set_mode_client_;
  as2::SynchronousServiceClient<mavros_msgs::srv::CommandLong>::SharedPtr
    mavlink_command_long_client_;
  as2::SynchronousServiceClient<mavros_msgs::srv::CommandTOL>::SharedPtr mavlink_takeoff_client_;
  as2::SynchronousServiceClient<mavros_msgs::srv::CommandTOL>::SharedPtr mavlink_land_client_;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr mavlink_setpoint_raw_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr mavlink_attitude_setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavlink_vision_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mavlink_vision_speed_pub_;
  rclcpp::TimerBase::SharedPtr vision_pose_timer_;

  // State
  bool has_mode_settled_ = false;
  bool hover_setpoint_latched_ = false;
  geometry_msgs::msg::PoseStamped hover_pose_;
  bool stop_requested_ = false;

  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;

  geometry_msgs::msg::PoseStamped mavlink_vision_pose_msg_;
  geometry_msgs::msg::TwistStamped mavlink_vision_speed_msg_;
};

}  // namespace as2_platform_mavlink

#endif  // AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_
