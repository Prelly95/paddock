// Paddock inspection mission sequencer.
//
// Drives a PX4 standard_vtol through a farm inspection sortie:
//
//   arm -> MC takeoff -> [depart toward paddock -> transition FW -> cruise
//   -> transition MC -> offboard hover & inspect] xN -> RTL
//
// Division of labour (the central design decision here): PX4's navigator owns
// guidance during fixed-wing cruise, commanded via DO_REPOSITION. This node
// takes direct offboard control only while in multirotor mode over a paddock.
// Streaming trajectory setpoints at a fixed-wing aircraft means re-deriving
// coordinated-turn guidance that PX4 already implements correctly, so the
// cruise legs stay with the navigator and the hover legs — where the actual
// inspection behaviour lives — are ours.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vtol_vehicle_status.hpp>

namespace paddock
{

using namespace std::chrono_literals;

// PX4 custom mode values for MAV_CMD_DO_SET_MODE. These live in PX4's
// commander (px4_custom_mode.h) rather than in any .msg file, so they have to
// be mirrored here.
constexpr float kModeFlagCustomEnabled = 1.0f;
constexpr float kCustomMainModeAuto = 4.0f;
constexpr float kCustomMainModeOffboard = 6.0f;
constexpr float kCustomSubModeAutoLoiter = 3.0f;
constexpr float kCustomSubModeAutoRtl = 5.0f;

// MAV_VTOL_STATE values used as param1 of DO_VTOL_TRANSITION.
constexpr float kVtolStateMc = 3.0f;
constexpr float kVtolStateFw = 4.0f;

// MAV_DO_REPOSITION_FLAGS bit 0. Without it PX4's commander takes the
// "mode switch not requested" branch, acks UNSUPPORTED and never switches to
// AUTO_LOITER. The navigator still consumes the coordinates, so the vehicle
// appears to comply whenever it is already loitering -- and silently ignores
// the command when it is not, e.g. on the way out of an offboard leg.
constexpr float kRepositionChangeMode = 1.0f;

constexpr double kEarthRadiusM = 6371000.0;

enum class Phase
{
  WaitForFcu,
  Arm,
  Takeoff,
  Depart,
  TransitionFw,
  Cruise,
  TransitionMc,
  Inspect,
  LeaveOffboard,
  Rtl,
  Done,
};

const char * phase_name(Phase p)
{
  switch (p) {
    case Phase::WaitForFcu:    return "WAIT_FOR_FCU";
    case Phase::Arm:           return "ARM";
    case Phase::Takeoff:       return "TAKEOFF";
    case Phase::Depart:        return "DEPART";
    case Phase::TransitionFw:  return "TRANSITION_FW";
    case Phase::Cruise:        return "CRUISE";
    case Phase::TransitionMc:  return "TRANSITION_MC";
    case Phase::Inspect:       return "INSPECT";
    case Phase::LeaveOffboard: return "LEAVE_OFFBOARD";
    case Phase::Rtl:           return "RTL";
    case Phase::Done:          return "DONE";
  }
  return "UNKNOWN";
}

const char * command_name(uint32_t cmd)
{
  using VC = px4_msgs::msg::VehicleCommand;
  switch (cmd) {
    case VC::VEHICLE_CMD_COMPONENT_ARM_DISARM: return "ARM_DISARM";
    case VC::VEHICLE_CMD_NAV_TAKEOFF:          return "NAV_TAKEOFF";
    case VC::VEHICLE_CMD_DO_VTOL_TRANSITION:   return "DO_VTOL_TRANSITION";
    case VC::VEHICLE_CMD_DO_REPOSITION:        return "DO_REPOSITION";
    case VC::VEHICLE_CMD_DO_SET_MODE:          return "DO_SET_MODE";
    default:                                   return "OTHER";
  }
}

const char * ack_result_name(uint8_t r)
{
  using ACK = px4_msgs::msg::VehicleCommandAck;
  switch (r) {
    case ACK::VEHICLE_CMD_RESULT_ACCEPTED:              return "ACCEPTED";
    case ACK::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:  return "TEMPORARILY_REJECTED";
    case ACK::VEHICLE_CMD_RESULT_DENIED:                return "DENIED";
    case ACK::VEHICLE_CMD_RESULT_UNSUPPORTED:           return "UNSUPPORTED";
    case ACK::VEHICLE_CMD_RESULT_FAILED:                return "FAILED";
    case ACK::VEHICLE_CMD_RESULT_IN_PROGRESS:           return "IN_PROGRESS";
    case ACK::VEHICLE_CMD_RESULT_CANCELLED:             return "CANCELLED";
    default:                                            return "UNKNOWN";
  }
}

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission_node")
  {
    declare_parameters();

    // PX4 publishes its uORB-backed topics as BEST_EFFORT / KEEP_LAST(5).
    // A default (RELIABLE) subscription is QoS-incompatible and silently
    // receives nothing — the single most common way a PX4 ROS 2 node appears
    // to connect fine while no callback ever fires.
    const auto px4_qos = rclcpp::SensorDataQoS();

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4_qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        status_ = *msg;
        have_status_ = true;
      });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        local_pos_ = *msg;
        have_local_pos_ = true;
      });

    vtol_sub_ = create_subscription<px4_msgs::msg::VtolVehicleStatus>(
      "/fmu/out/vtol_vehicle_status", px4_qos,
      [this](px4_msgs::msg::VtolVehicleStatus::SharedPtr msg) {
        vtol_ = *msg;
        have_vtol_ = true;
      });

    // Without this, every command is fire-and-forget: a rejected DO_REPOSITION
    // is indistinguishable from an accepted one that the vehicle ignored.
    ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", px4_qos,
      [this](px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
        const bool ok =
          msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
        if (ok) {
          RCLCPP_INFO(
            get_logger(), "ack: %s ACCEPTED", command_name(msg->command));
        } else {
          RCLCPP_WARN(
            get_logger(), "ack: %s -> %s (reason=%u)",
            command_name(msg->command), ack_result_name(msg->result),
            msg->result_param1);
        }
      });

    // Commands out to PX4. Default (reliable) QoS is fine here: a reliable
    // publisher can serve PX4's best-effort subscriptions.
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);

    // Outward-facing mission state, consumed by the inspection node and the
    // C2 dashboard. Latched so a dashboard connecting mid-sortie sees the
    // current phase immediately rather than waiting for the next change.
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "paddock/mission_state", rclcpp::QoS(1).transient_local());
    inspect_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "paddock/inspect_trigger", 10);

    timer_ = create_wall_timer(50ms, [this] { tick(); });

    RCLCPP_INFO(
      get_logger(), "Mission node up: %zu paddock(s), cruise %.0f m, inspect %.0f m",
      paddocks_n_.size(), cruise_alt_, inspect_alt_);
    publish_state();
  }

private:
  // ---------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------
  void declare_parameters()
  {
    // Paddock positions as local NE offsets from the launch point, in metres.
    // One source of truth: the lat/lon handed to DO_REPOSITION is derived
    // from these at runtime using PX4's own local origin.
    paddocks_n_ = declare_parameter<std::vector<double>>(
      "paddocks_north_m", {400.0, 800.0, 300.0});
    paddocks_e_ = declare_parameter<std::vector<double>>(
      "paddocks_east_m", {200.0, -300.0, -700.0});

    takeoff_alt_ = declare_parameter<double>("takeoff_alt_m", 40.0);
    cruise_alt_ = declare_parameter<double>("cruise_alt_m", 80.0);
    inspect_alt_ = declare_parameter<double>("inspect_alt_m", 30.0);

    // Must exceed PX4's NAV_LOITER_RAD (80 m by default). DO_REPOSITION puts
    // a fixed-wing into an orbit *around* the target, so its ground track
    // never gets closer than the loiter radius. A tighter acceptance radius
    // than that leaves the aircraft circling a paddock forever, waiting on a
    // condition that can't be met.
    accept_radius_ = declare_parameter<double>("accept_radius_m", 120.0);

    // Tolerance for "settled over the paddock" once in offboard hover.
    // Groundspeed required before a front transition is commanded.
    //
    // This must sit BELOW the multirotor cruise speed, not above the airspeed
    // the transition needs: in AUTO modes a multirotor tracks MPC_XY_CRUISE
    // (5 m/s by default, hard-capped by MPC_XY_VEL_MAX at 8), so any threshold
    // at or above that is simply unreachable and the departure never ends.
    // The point is only to leave the hover with real momentum -- entering the
    // transition at ~5 m/s clears VT_ARSP_BLEND (8 m/s) inside the
    // VT_F_TRANS_DUR window comfortably, where starting from 0 m/s peaked at
    // 6.1 m/s and quad-chuted.
    transition_speed_ = declare_parameter<double>("transition_speed_m_s", 4.5);
    // How closely the nose must point at the paddock before transitioning.
    // A multirotor translates in any direction regardless of heading, but the
    // pusher accelerates along body-X and the pitot measures along body-X --
    // so groundspeed only becomes useful airspeed to the extent the nose is
    // aligned with it. Crabbing 51 degrees off course, as measured, throws
    // away cos(51) = 37% of the departure speed.
    heading_tolerance_ = declare_parameter<double>("heading_tolerance_deg", 15.0);
    hover_tolerance_ = declare_parameter<double>("hover_tolerance_m", 3.0);
    dwell_s_ = declare_parameter<double>("inspect_dwell_s", 10.0);
    cmd_retry_s_ = declare_parameter<double>("command_retry_s", 2.0);

    if (paddocks_n_.size() != paddocks_e_.size() || paddocks_n_.empty()) {
      RCLCPP_FATAL(
        get_logger(),
        "paddocks_north_m (%zu) and paddocks_east_m (%zu) must be non-empty and equal length",
        paddocks_n_.size(), paddocks_e_.size());
      throw std::runtime_error("invalid paddock waypoint configuration");
    }
  }

  // ---------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------
  uint64_t px4_timestamp() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void send_command(
    uint32_t command, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f,
    float p4 = 0.0f, double p5 = 0.0, double p6 = 0.0, float p7 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = px4_timestamp();
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.param3 = p3;
    msg.param4 = p4;
    msg.param5 = p5;
    msg.param6 = p6;
    msg.param7 = p7;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    cmd_pub_->publish(msg);
  }

  // Equirectangular projection about the local origin. Good to well under a
  // metre over the few-km scale of a paddock circuit, and it keeps the
  // waypoint list in the units a farm is actually described in.
  void local_to_global(double north, double east, double & lat, double & lon) const
  {
    const double ref_lat_rad = local_pos_.ref_lat * M_PI / 180.0;
    lat = local_pos_.ref_lat + (north / kEarthRadiusM) * 180.0 / M_PI;
    lon = local_pos_.ref_lon +
      (east / (kEarthRadiusM * std::cos(ref_lat_rad))) * 180.0 / M_PI;
  }

  // Bearing from the vehicle to a local NE point, radians in NED (0 = north).
  double bearing_to(double north, double east) const
  {
    return std::atan2(east - local_pos_.y, north - local_pos_.x);
  }

  // Absolute angle between where the nose points and where the target is.
  double heading_error_to(double north, double east) const
  {
    double err = bearing_to(north, east) - local_pos_.heading;
    while (err > M_PI) {err -= 2.0 * M_PI;}
    while (err < -M_PI) {err += 2.0 * M_PI;}
    return std::abs(err);
  }

  double groundspeed() const
  {
    return std::hypot(local_pos_.vx, local_pos_.vy);
  }

  double horizontal_distance_to(double north, double east) const
  {
    const double dn = north - local_pos_.x;
    const double de = east - local_pos_.y;
    return std::hypot(dn, de);
  }

  // True once on phase entry, then every command_retry_s. PX4 rejects
  // commands that arrive before it is ready to act on them (mid-transition,
  // pre-arm checks still failing), and does so without any retry of its own,
  // so every command in this node is re-sent until its effect is observed.
  bool should_send_command()
  {
    const auto now = get_clock()->now();
    if (!command_sent_) {
      command_sent_ = true;
      last_command_ = now;
      return true;
    }
    if ((now - last_command_).seconds() >= cmd_retry_s_) {
      last_command_ = now;
      return true;
    }
    return false;
  }

  void set_phase(Phase next)
  {
    RCLCPP_INFO(get_logger(), "%s -> %s", phase_name(phase_), phase_name(next));
    phase_ = next;
    phase_entered_ = get_clock()->now();
    command_sent_ = false;
    publish_state();
  }

  double seconds_in_phase() const
  {
    return (get_clock()->now() - phase_entered_).seconds();
  }

  void publish_state()
  {
    std_msgs::msg::String msg;
    msg.data = phase_name(phase_);
    if (phase_ == Phase::Cruise || phase_ == Phase::TransitionMc ||
      phase_ == Phase::Inspect)
    {
      msg.data += " paddock=" + std::to_string(wp_index_ + 1) + "/" +
        std::to_string(paddocks_n_.size());
    }
    state_pub_->publish(msg);
  }

  // Offboard requires a setpoint stream that is already flowing before the
  // mode switch, and that never gaps by more than ~0.5 s afterwards, or the
  // vehicle drops out of offboard into a failsafe.
  void stream_offboard_setpoint(double north, double east, double alt_m, float yaw)
  {
    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = px4_timestamp();
    mode.position = true;
    offboard_mode_pub_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = mode.timestamp;
    sp.position = {
      static_cast<float>(north), static_cast<float>(east),
      static_cast<float>(-alt_m)};  // NED: down is positive, so altitude negates
    sp.yaw = yaw;
    setpoint_pub_->publish(sp);
  }

  // ---------------------------------------------------------------------
  // State machine
  // ---------------------------------------------------------------------
  void tick()
  {
    switch (phase_) {
      case Phase::WaitForFcu:    run_wait_for_fcu();    break;
      case Phase::Arm:           run_arm();             break;
      case Phase::Takeoff:       run_takeoff();         break;
      case Phase::Depart:        run_depart();          break;
      case Phase::TransitionFw:  run_transition_fw();   break;
      case Phase::Cruise:        run_cruise();          break;
      case Phase::TransitionMc:  run_transition_mc();   break;
      case Phase::Inspect:       run_inspect();         break;
      case Phase::LeaveOffboard: run_leave_offboard();  break;
      case Phase::Rtl:           run_rtl();             break;
      case Phase::Done:                                 break;
    }
  }

  void run_wait_for_fcu()
  {
    if (!have_status_ || !have_local_pos_ || !have_vtol_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for PX4 telemetry (status=%d pos=%d vtol=%d) — is the uXRCE-DDS agent up?",
        have_status_, have_local_pos_, have_vtol_);
      return;
    }
    // xy_global gates on ref_lat/ref_lon being populated, which is what the
    // DO_REPOSITION waypoints are derived from. Arming before the estimator
    // has a global reference means cruising to a garbage coordinate.
    if (!local_pos_.xy_valid || !local_pos_.xy_global) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for EKF2 global reference (xy_valid=%d xy_global=%d)",
        local_pos_.xy_valid, local_pos_.xy_global);
      return;
    }
    home_alt_ = local_pos_.ref_alt;
    RCLCPP_INFO(
      get_logger(), "EKF2 ready. Origin %.7f, %.7f at %.1f m AMSL",
      local_pos_.ref_lat, local_pos_.ref_lon, home_alt_);
    set_phase(Phase::Arm);
  }

  void run_arm()
  {
    if (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      set_phase(Phase::Takeoff);
      return;
    }
    if (should_send_command()) {
      send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    }
  }

  void run_takeoff()
  {
    // Deliberately NAV_TAKEOFF rather than NAV_VTOL_TAKEOFF. The VTOL variant
    // climbs *and* transitions to fixed-wing as one opaque action; the plain
    // takeoff leaves the vehicle in multirotor mode so the transition stays
    // an explicit, observable step of this state machine.
    if (should_send_command()) {
      double lat, lon;
      local_to_global(0.0, 0.0, lat, lon);
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
        0.0f, 0.0f, 0.0f, NAN, lat, lon,
        static_cast<float>(home_alt_ + takeoff_alt_));
    }
    const double altitude = -local_pos_.z;
    if (altitude >= takeoff_alt_ * 0.95) {
      RCLCPP_INFO(get_logger(), "Takeoff complete at %.1f m AGL", altitude);
      set_phase(Phase::Depart);
    }
  }

  // A front transition commanded from a stationary hover does not survive.
  // PX4 allows VT_F_TRANS_DUR (5 s) to reach VT_ARSP_BLEND (8 m/s) and
  // quad-chutes the instant that window closes short. Starting from a dead
  // hover in AUTO_LOITER, the multirotor position controller resists the
  // forward acceleration and the airframe arrives at the deadline ~2 m/s
  // light — measured at 6.1 m/s against the 8.0 m/s requirement.
  //
  // So reposition toward the paddock while still in multirotor mode and let
  // the position controller build forward speed first. The transition then
  // starts with momentum already established, which is the condition a normal
  // PX4 VTOL mission transitions under.
  void run_depart()
  {
    const double north = paddocks_n_[wp_index_];
    const double east = paddocks_e_[wp_index_];

    // Nothing to depart for if we are already there. Without this the phase
    // is degenerate: distance ~0 makes bearing_to() the atan2 of position
    // noise, so the commanded yaw is random, speed never builds, and neither
    // gate condition can ever be met -- a permanent stall rather than a
    // failure. Happens whenever a sortie is resumed over a paddock.
    if (horizontal_distance_to(north, east) <= accept_radius_) {
      RCLCPP_INFO(
        get_logger(), "Already within %.0f m of paddock %zu — skipping cruise",
        accept_radius_, wp_index_ + 1);
      set_phase(Phase::TransitionMc);
      return;
    }

    if (should_send_command()) {
      double lat, lon;
      local_to_global(north, east, lat, lon);
      // param4 is the yaw setpoint, and it must be in RADIANS despite the
      // MAVLink spec defining it as degrees. PX4's navigator assigns it
      // straight into position_setpoint_s::yaw, which is radians, with no
      // conversion (navigator_main.cpp, the DO_REPOSITION branch) -- unlike
      // other commands that call math::radians() explicitly. Sending degrees
      // silently wraps: a 26.57 deg bearing became 26.57 rad = 82.1 deg, and
      // the aircraft held that heading for an entire leg, crabbing ~56 deg
      // off course while faithfully obeying the command it was given.
      //
      // Nothing catches this: the value is finite and in range, so it is
      // accepted and acted on. It is only visible by comparing commanded
      // heading against achieved heading.
      const double bearing_rad = bearing_to(north, east);
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION,
        -1.0f, kRepositionChangeMode, 0.0f,
        static_cast<float>(bearing_rad),
        lat, lon,
        static_cast<float>(home_alt_ + cruise_alt_));
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Departing for paddock %zu | speed %.1f of %.1f m/s | heading err "
        "%.0f of %.0f deg | pos=(%.0f,%.0f) nav_state=%u",
        wp_index_ + 1, groundspeed(), transition_speed_,
        heading_error_to(north, east) * 180.0 / M_PI, heading_tolerance_,
        local_pos_.x, local_pos_.y, status_.nav_state);
    }

    // Both conditions matter: speed alone is useless if it is sideways.
    const double heading_err_deg = heading_error_to(north, east) * 180.0 / M_PI;
    if (groundspeed() >= transition_speed_ && heading_err_deg <= heading_tolerance_) {
      RCLCPP_INFO(
        get_logger(), "Departure: %.1f m/s, nose %.0f deg off — transitioning",
        groundspeed(), heading_err_deg);
      set_phase(Phase::TransitionFw);
    }
  }

  void run_transition_fw()
  {
    if (vtol_.vehicle_vtol_state == px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW) {
      set_phase(Phase::Cruise);
      return;
    }
    if (should_send_command()) {
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
        kVtolStateFw, 0.0f);
    }
  }

  void run_cruise()
  {
    const double north = paddocks_n_[wp_index_];
    const double east = paddocks_e_[wp_index_];

    if (should_send_command()) {
      double lat, lon;
      local_to_global(north, east, lat, lon);
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION,
        -1.0f,                   // ground speed: -1 = configured cruise speed
        kRepositionChangeMode,   // flags: allow the required mode switch
        0.0f,    // loiter radius: 0 = NAV_LOITER_RAD
        NAN,     // yaw: unconstrained
        lat, lon,
        static_cast<float>(home_alt_ + cruise_alt_));
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cruising to paddock %zu: %.0f m out",
        wp_index_ + 1, horizontal_distance_to(north, east));
    }

    if (horizontal_distance_to(north, east) <= accept_radius_) {
      RCLCPP_INFO(get_logger(), "Reached paddock %zu", wp_index_ + 1);
      set_phase(Phase::TransitionMc);
    }
  }

  void run_transition_mc()
  {
    if (vtol_.vehicle_vtol_state == px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC) {
      inspect_hold_yaw_ = local_pos_.heading;
      offboard_ticks_ = 0;
      dwell_started_ = false;
      set_phase(Phase::Inspect);
      return;
    }
    if (should_send_command()) {
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
        kVtolStateMc, 0.0f);
    }
  }

  void run_inspect()
  {
    const double north = paddocks_n_[wp_index_];
    const double east = paddocks_e_[wp_index_];

    stream_offboard_setpoint(north, east, inspect_alt_, inspect_hold_yaw_);

    // PX4 will not accept the offboard mode switch until it has seen a
    // setpoint stream. Give it ~0.5 s of history before asking.
    if (++offboard_ticks_ == 10) {
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        kModeFlagCustomEnabled, kCustomMainModeOffboard);
    }
    if (offboard_ticks_ < 10 ||
      status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      return;
    }

    const double horizontal_err = horizontal_distance_to(north, east);
    const double vertical_err = std::abs(-local_pos_.z - inspect_alt_);
    const bool settled = horizontal_err <= hover_tolerance_ &&
      vertical_err <= hover_tolerance_;

    if (!dwell_started_) {
      if (!settled) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Settling over paddock %zu (h=%.1f m, v=%.1f m)",
          wp_index_ + 1, horizontal_err, vertical_err);
        return;
      }
      dwell_started_ = true;
      dwell_start_ = get_clock()->now();

      // Hand off to the inspection node. Fire-and-forget by design: the
      // mission must not stall if nothing is listening.
      std_msgs::msg::UInt8 trigger;
      trigger.data = static_cast<uint8_t>(wp_index_ + 1);
      inspect_pub_->publish(trigger);
      RCLCPP_INFO(
        get_logger(), "On station over paddock %zu — inspecting for %.0f s",
        wp_index_ + 1, dwell_s_);
      return;
    }

    if ((get_clock()->now() - dwell_start_).seconds() >= dwell_s_) {
      set_phase(Phase::LeaveOffboard);
    }
  }

  void run_leave_offboard()
  {
    // Keep the setpoint stream alive until PX4 has actually left offboard,
    // otherwise the gap trips a failsafe on the way out.
    stream_offboard_setpoint(
      paddocks_n_[wp_index_], paddocks_e_[wp_index_], inspect_alt_, inspect_hold_yaw_);

    if (status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
      if (++wp_index_ < paddocks_n_.size()) {
        set_phase(Phase::Depart);
      } else {
        RCLCPP_INFO(get_logger(), "All %zu paddocks inspected", paddocks_n_.size());
        set_phase(Phase::Rtl);
      }
      return;
    }
    if (should_send_command()) {
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        kModeFlagCustomEnabled, kCustomMainModeAuto, kCustomSubModeAutoLoiter);
    }
  }

  void run_rtl()
  {
    // RTL rather than an explicit cruise-home-and-land sequence: for a VTOL,
    // PX4's RTL already handles the transition back to multirotor and the
    // descent, and reusing it keeps the return leg on the same well-tested
    // path a real deployment would use.
    if (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
      seconds_in_phase() > 5.0)
    {
      RCLCPP_INFO(get_logger(), "Landed and disarmed. Mission complete.");
      set_phase(Phase::Done);
      return;
    }
    if (should_send_command()) {
      send_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        kModeFlagCustomEnabled, kCustomMainModeAuto, kCustomSubModeAutoRtl);
    }
  }

  // ---------------------------------------------------------------------
  std::vector<double> paddocks_n_, paddocks_e_;
  double takeoff_alt_{}, cruise_alt_{}, inspect_alt_{};
  double accept_radius_{}, hover_tolerance_{}, dwell_s_{}, cmd_retry_s_{};
  double transition_speed_{};
  double heading_tolerance_{};

  px4_msgs::msg::VehicleStatus status_{};
  px4_msgs::msg::VehicleLocalPosition local_pos_{};
  px4_msgs::msg::VtolVehicleStatus vtol_{};
  bool have_status_{false}, have_local_pos_{false}, have_vtol_{false};

  Phase phase_{Phase::WaitForFcu};
  rclcpp::Time phase_entered_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time dwell_start_{0, 0, RCL_ROS_TIME};
  bool command_sent_{false};
  bool dwell_started_{false};
  size_t wp_index_{0};
  int offboard_ticks_{0};
  float inspect_hold_yaw_{0.0f};
  double home_alt_{0.0};

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VtolVehicleStatus>::SharedPtr vtol_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr inspect_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace paddock

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<paddock::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
