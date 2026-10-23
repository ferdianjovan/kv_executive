// Observe measured EXECUTE_PATH battery consumption without issuing flight commands.
// Keep this passive node outside CPython/rclpy's executor runtime.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kv_executive/msg/battery_return_estimate.hpp>
#include <kv_executive/msg/battery_usage.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace kv_executive
{

class BatteryMonitor final : public rclcpp::Node
{
public:
  BatteryMonitor()
  : Node("battery_monitor")
  {
    battery_topic_ = string_parameter("battery_topic", "/mavros/battery");
    odometry_topic_ = string_parameter("local_odometry_topic", "/mavros/local_position/odom");
    state_topic_ = string_parameter("vehicle_state_topic", "/mavros/state");
    status_topic_ = string_parameter("mission_status_topic", "mission_status");
    home_topic_ = string_parameter("home_position_topic", "/mavros/home_position/home");
    local_frame_ = string_parameter("local_frame", "px4_odom");
    battery_timeout_ = positive_parameter("battery_timeout", 5.0);
    odometry_timeout_ = positive_parameter("odometry_timeout", 2.0);
    state_timeout_ = positive_parameter("state_timeout", 10.0);
    minimum_distance_ = positive_parameter("minimum_sample_distance", 1.0);
    reserve_percent_ = declare_parameter<double>("reserve_percent", 15.0);
    validate_percentage(reserve_percent_, "reserve_percent");
    safety_factor_ = positive_parameter("return_safety_factor", 1.3);
    if (safety_factor_ < 1.0) {
      throw std::invalid_argument("return_safety_factor must be at least one");
    }
    alert_repeat_period_ = positive_parameter("alert_repeat_period", 10.0);
    const double monitor_rate = positive_parameter("monitor_rate", 2.0);

    usage_publisher_ = create_publisher<Usage>(
      "battery_usage", rclcpp::QoS(100).transient_local());
    estimate_publisher_ = create_publisher<Estimate>("battery_return_estimate", 10);
    battery_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::BatteryState::ConstSharedPtr message) {
        battery_callback(*message);
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odometry_callback(*message);
      });
    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, rclcpp::QoS(1).transient_local(),
      [this](const mavros_msgs::msg::State::ConstSharedPtr message) {
        state_callback(*message);
      });
    home_subscription_ = create_subscription<mavros_msgs::msg::HomePosition>(
      home_topic_, rclcpp::QoS(1).transient_local(),
      [this](const mavros_msgs::msg::HomePosition::ConstSharedPtr message) {
        home_callback(*message);
      });
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(100).transient_local(),
      [this](const std_msgs::msg::String::ConstSharedPtr message) {
        status_callback(message->data);
      });
    // Use the ROS clock for scheduling, preserving /clock and use_sim_time.
    // Freshness and alert throttling deliberately use the steady wall clock.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / monitor_rate),
      [this]() {tick();});
    RCLCPP_INFO(
      get_logger(), "Battery monitor ready (C++ runtime); waiting for EXECUTE_PATH");
  }

private:
  using Pose = geometry_msgs::msg::PoseStamped;
  using Usage = kv_executive::msg::BatteryUsage;
  using Estimate = kv_executive::msg::BatteryReturnEstimate;

  struct Segment
  {
    Pose start_pose;
    Pose end_pose;
    double start_percent;
    double end_percent;
    double distance_m = 0.0;
    std::string invalid_reason;
  };

  struct Record
  {
    Usage message;
    std::string invalid_reason;
  };

  struct Baseline
  {
    Pose pose;
    double battery_percent;
  };

  static double monotonic()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static bool starts_with(const std::string & text, const std::string & prefix)
  {
    return text.compare(0, prefix.size(), prefix) == 0;
  }

  static bool nonempty(const std::string & value)
  {
    return std::any_of(
      value.begin(), value.end(), [](unsigned char c) {return !std::isspace(c);});
  }

  std::string string_parameter(const std::string & name, const std::string & default_value)
  {
    auto value = declare_parameter<std::string>(name, default_value);
    if (!nonempty(value)) {
      throw std::invalid_argument(name + " must not be empty");
    }
    return value;
  }

  double positive_parameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  static void validate_percentage(double value, const std::string & description)
  {
    if (!std::isfinite(value) || value < 0.0 || value > 100.0) {
      throw std::invalid_argument(description + " must be between 0 and 100");
    }
  }

  static void validate_pose(const Pose & pose)
  {
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    if (!nonempty(pose.header.frame_id)) {
      throw std::invalid_argument("Pose must have a nonempty coordinate frame");
    }
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w))
    {
      throw std::invalid_argument("Pose coordinates and quaternion must be finite");
    }
    if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0) {
      throw std::invalid_argument("Pose quaternion must have a nonzero norm");
    }
  }

  bool valid_local_pose(const Pose & pose) const
  {
    if (pose.header.frame_id != local_frame_) {
      return false;
    }
    try {
      validate_pose(pose);
      return true;
    } catch (const std::invalid_argument &) {
      return false;
    }
  }

  static double distance(const Pose & first, const Pose & second)
  {
    const auto & a = first.pose.position;
    const auto & b = second.pose.position;
    return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  void info_once(const std::string & reason)
  {
    if (reason != unavailable_) {
      RCLCPP_INFO(get_logger(), "Battery monitor: %s", reason.c_str());
    }
    unavailable_ = reason;
  }

  void invalidate(const std::string & reason)
  {
    if (active_) {
      if (active_->invalid_reason.empty()) {
        active_->invalid_reason = reason.empty() ? "Invalid telemetry" : reason;
      }
      interval_invalid_ = true;
    }
    if (phase_active_) {
      info_once(reason);
    }
  }

  void check_gap(const std::optional<double> & received, double timeout, const std::string & name)
  {
    if (received && monotonic() - *received >= timeout) {
      invalidate(name + " telemetry gap; this interval cannot teach the model");
    }
  }

  bool odometry_fresh() const
  {
    return odometry_received_ && monotonic() - *odometry_received_ < odometry_timeout_;
  }

  std::string telemetry_problem(bool require_armed = true) const
  {
    const double now = monotonic();
    if (!battery_percent_ || !battery_received_) {
      return "Waiting for a valid MAVROS battery percentage";
    }
    if (now - *battery_received_ >= battery_timeout_) {
      return "Battery telemetry is stale";
    }
    if (!pose_ || !odometry_fresh()) {
      return "Local odometry is unavailable or stale";
    }
    if (!vehicle_state_ || !state_received_ || now - *state_received_ >= state_timeout_ ||
      !vehicle_state_->connected)
    {
      return "Vehicle state is unavailable, stale, or disconnected";
    }
    if (require_armed && !vehicle_state_->armed) {
      return "Vehicle is disarmed";
    }
    return "";
  }

  bool establish_baseline()
  {
    if (!telemetry_problem().empty()) {
      return false;
    }
    baseline_ = Baseline{*pose_, *battery_percent_};
    return true;
  }

  void battery_callback(const sensor_msgs::msg::BatteryState & message)
  {
    check_gap(battery_received_, battery_timeout_, "Battery");
    battery_received_ = monotonic();
    const double value = message.percentage;
    if (!message.present || !std::isfinite(value) || value < 0.0 || value > 1.0 ||
      message.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
    {
      battery_percent_.reset();
      invalidate("Battery percentage unavailable, invalid, or charging");
      return;
    }
    battery_percent_ = value * 100.0;
    observe();
  }

  void odometry_callback(const nav_msgs::msg::Odometry & message)
  {
    check_gap(odometry_received_, odometry_timeout_, "Odometry");
    odometry_received_ = monotonic();
    Pose pose;
    pose.header = message.header;
    pose.pose = message.pose.pose;
    if (!valid_local_pose(pose)) {
      pose_.reset();
      invalidate("Invalid odometry or unexpected local frame");
      return;
    }
    pose_ = pose;
    if (vehicle_state_ && !vehicle_state_->armed && vehicle_state_->connected &&
      !home_locked_ && state_received_ && monotonic() - *state_received_ < state_timeout_)
    {
      ground_pose_ = pose;
      home_pose_ = pose;
    }
    observe();
  }

  void state_callback(const mavros_msgs::msg::State & message)
  {
    check_gap(state_received_, state_timeout_, "Vehicle state");
    const auto previous = vehicle_state_;
    vehicle_state_ = message;
    state_received_ = monotonic();
    if (!message.connected) {
      invalidate("FCU disconnected");
    }
    if (!message.armed) {
      if (phase_active_) {
        finish_phase("disarmed");
      }
      baseline_.reset();
      if (previous && previous->armed) {
        home_locked_ = false;
      }
      if (!home_locked_ && message.connected && pose_ && odometry_fresh()) {
        ground_pose_ = pose_;
        home_pose_ = pose_;
      }
    } else if (!previous || !previous->armed) {
      if (ground_pose_) {
        home_pose_ = ground_pose_;
      } else if (!home_pose_) {
        home_pose_ = mavros_home_;
      }
    }
    observe();
  }

  void home_callback(const mavros_msgs::msg::HomePosition & message)
  {
    // MAVROS HOME_POSITION.position already converts PX4 local NED to ENU.
    // Its numeric position uses the local estimator origin despite its header.
    Pose pose;
    pose.header = message.header;
    pose.header.frame_id = local_frame_;
    pose.pose.position = message.position;
    pose.pose.orientation.w = 1.0;
    if (!valid_local_pose(pose)) {
      return;
    }
    mavros_home_ = pose;
    if (!ground_pose_ && !home_pose_) {
      home_pose_ = pose;
    }
  }

  void status_callback(const std::string & text)
  {
    if (text == "TAKEOFF: started") {
      baseline_.reset();
      // Freeze ground home before slower armed-state delivery catches up
      // with the first climbing odometry samples.
      if (ground_pose_) {
        home_pose_ = ground_pose_;
        home_locked_ = true;
      }
    } else if (text == "TAKEOFF: done") {
      if (establish_baseline()) {
        RCLCPP_INFO(get_logger(), "Battery after takeoff: %.2f%%", *battery_percent_);
      }
    } else if (text == "EXECUTE_PATH: started") {
      if (phase_active_) {
        return;
      }
      phase_active_ = true;
      interval_invalid_ = false;
      alert_active_ = false;
      last_alert_ = -std::numeric_limits<double>::infinity();
      if (establish_baseline()) {
        active_ = Segment{*pose_, *pose_, *battery_percent_, *battery_percent_, 0.0, ""};
        last_good_pose_ = pose_;
        last_good_percent_ = battery_percent_;
        RCLCPP_INFO(get_logger(), "EXECUTE_PATH battery baseline: %.2f%%", *battery_percent_);
      } else {
        info_once(
          "No fresh baseline at EXECUTE_PATH start; skipping learning for this interval");
      }
    } else if (text == "EXECUTE_PATH: done") {
      finish_phase("completed");
    } else if (starts_with(text, "MISSION: canceled") || starts_with(text, "MISSION: aborted")) {
      finish_phase(starts_with(text, "MISSION: canceled") ? "canceled" : "aborted");
      if (vehicle_state_ && vehicle_state_->connected && !vehicle_state_->armed) {
        home_locked_ = false;
        if (pose_ && odometry_fresh()) {
          ground_pose_ = pose_;
          home_pose_ = pose_;
        }
      }
    } else if (phase_active_ && starts_with(text, "EXECUTE_PATH:") &&
      !starts_with(text, "EXECUTE_PATH: sub-path"))
    {
      finish_phase("aborted");
    }
  }

  void update_segment(const Pose & pose, double battery_percent)
  {
    if (!active_) {
      throw std::logic_error("No battery usage segment is active");
    }
    validate_pose(pose);
    validate_percentage(battery_percent, "Battery percentage");
    if (pose.header.frame_id != active_->start_pose.header.frame_id) {
      throw std::invalid_argument("WALK poses must use one coordinate frame");
    }
    if (battery_percent > active_->end_percent + 0.1) {
      throw std::invalid_argument("Battery percentage increased during WALK");
    }
    const double travelled = active_->distance_m + distance(active_->end_pose, pose);
    if (!std::isfinite(travelled)) {
      throw std::invalid_argument("Travelled distance must be finite");
    }
    active_->distance_m = travelled;
    active_->end_pose = pose;
    active_->end_percent = std::min(active_->end_percent, battery_percent);
  }

  Record make_record(const Segment & segment, uint64_t sample_id, const std::string & outcome) const
  {
    Record record;
    auto & message = record.message;
    message.header = segment.end_pose.header;
    message.sample_id = sample_id;
    message.start_pose = segment.start_pose;
    message.end_pose = segment.end_pose;
    message.distance_m = segment.distance_m;
    message.start_battery_percent = segment.start_percent;
    message.end_battery_percent = segment.end_percent;
    message.consumed_percent = std::max(0.0, segment.start_percent - segment.end_percent);
    message.percentage_per_meter = segment.distance_m > 0.0 ?
      message.consumed_percent / segment.distance_m : 0.0;
    message.outcome = outcome;
    record.invalid_reason = segment.invalid_reason;
    if (record.invalid_reason.empty() && segment.distance_m < minimum_distance_) {
      record.invalid_reason = "Travelled distance is below the learning minimum";
    }
    if (record.invalid_reason.empty() && message.consumed_percent <= 0.0) {
      record.invalid_reason = "No measurable battery depletion";
    }
    message.valid_for_estimate = record.invalid_reason.empty();
    return record;
  }

  void observe()
  {
    if (!phase_active_ || !active_ || interval_invalid_) {
      return;
    }
    const std::string reason = telemetry_problem();
    if (!reason.empty()) {
      invalidate(reason);
      return;
    }
    try {
      update_segment(*pose_, *battery_percent_);
    } catch (const std::invalid_argument & error) {
      invalidate(error.what());
      return;
    }
    last_good_pose_ = pose_;
    last_good_percent_ = battery_percent_;
  }

  void finish_phase(const std::string & outcome)
  {
    if (!phase_active_) {
      return;
    }
    observe();
    phase_active_ = false;
    alert_active_ = false;
    if (!active_) {
      return;
    }
    // Retain the last trustworthy endpoint after any telemetry failure.
    // No post-gap movement or consumption is invented for the stored record.
    update_segment(*last_good_pose_, *last_good_percent_);
    const Record record = make_record(*active_, next_sample_id_, outcome);
    records_.emplace(next_sample_id_, record);
    ++next_sample_id_;
    active_.reset();
    usage_publisher_->publish(record.message);
    const auto & message = record.message;
    const std::string exclusion = record.invalid_reason.empty() ? "" :
      "; excluded: " + record.invalid_reason;
    RCLCPP_INFO(
      get_logger(),
      "EXECUTE_PATH battery sample %llu: %.2f m, %.2f percentage points consumed; "
      "usable=%s (%s)%s",
      static_cast<unsigned long long>(message.sample_id), message.distance_m,
      message.consumed_percent, message.valid_for_estimate ? "True" : "False",
      outcome.c_str(), exclusion.c_str());
  }

  Estimate estimate() const
  {
    validate_pose(*pose_);
    validate_pose(*home_pose_);
    validate_percentage(*battery_percent_, "Battery percentage");
    if (pose_->header.frame_id != home_pose_->header.frame_id) {
      throw std::invalid_argument("Current and home poses must use one frame");
    }
    Estimate result;
    result.header.stamp = get_clock()->now();
    result.header.frame_id = local_frame_;
    result.current_pose = *pose_;
    result.home_pose = *home_pose_;
    result.remaining_percent = *battery_percent_;
    result.home_distance_m = distance(*pose_, *home_pose_);
    if (!std::isfinite(result.home_distance_m)) {
      throw std::invalid_argument("Home distance must be finite");
    }
    // Long-double accumulation limits error while preserving distance-weighted
    // learning across finished samples and at most one ongoing sample.
    long double consumed = 0.0L;
    long double travelled = 0.0L;
    const auto add_sample = [&consumed, &travelled, &result](const Record & record) {
        if (record.message.valid_for_estimate) {
          consumed += record.message.consumed_percent;
          travelled += record.message.distance_m;
          ++result.sample_count;
        }
      };
    for (const auto & item : records_) {
      add_sample(item.second);
    }
    if (active_) {
      add_sample(make_record(*active_, 0, "in_progress"));
    }
    result.estimate_available = result.sample_count > 0;
    const double unknown = std::numeric_limits<double>::quiet_NaN();
    result.mean_percent_per_meter = result.estimate_available ?
      static_cast<double>(consumed / travelled) : unknown;
    result.return_required_percent = result.estimate_available ?
      result.home_distance_m * result.mean_percent_per_meter : unknown;
    result.buffered_required_percent = reserve_percent_;
    if (result.estimate_available) {
      result.buffered_required_percent += result.return_required_percent * safety_factor_;
    }
    result.return_margin_percent = result.remaining_percent - result.buffered_required_percent;
    result.return_alert = result.remaining_percent <= result.buffered_required_percent;
    return result;
  }

  void tick()
  {
    if (!phase_active_) {
      return;
    }
    observe();
    const std::string reason = telemetry_problem();
    if (!reason.empty()) {
      invalidate(reason);
      return;
    }
    if (!home_pose_) {
      info_once("Home unavailable; waiting for ground odometry or MAVROS home");
      return;
    }
    Estimate message;
    try {
      message = estimate();
    } catch (const std::invalid_argument & error) {
      info_once(std::string("Return estimate unavailable: ") + error.what());
      return;
    }
    estimate_publisher_->publish(message);
    if (!message.estimate_available) {
      info_once("Learning battery use per metre; return estimate not available yet");
    } else {
      unavailable_.clear();
    }
    const double now = monotonic();
    if (message.return_alert && (!alert_active_ || now - last_alert_ >= alert_repeat_period_)) {
      if (message.estimate_available) {
        RCLCPP_INFO(
          get_logger(), "BATTERY RETURN ALERT: %.2f%% remaining; straight-line home distance "
          "%.2f m; return needs %.2f points, %.2f with safety buffer; margin %.2f points",
          *battery_percent_, message.home_distance_m, message.return_required_percent,
          message.buffered_required_percent, message.return_margin_percent);
      } else {
        RCLCPP_INFO(
          get_logger(), "BATTERY RESERVE ALERT: %.2f%% remaining; reserve %.2f%%; "
          "return rate still unknown", *battery_percent_, reserve_percent_);
      }
      last_alert_ = now;
    }
    alert_active_ = message.return_alert;
  }

  std::string battery_topic_;
  std::string odometry_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string home_topic_;
  std::string local_frame_;
  double battery_timeout_;
  double odometry_timeout_;
  double state_timeout_;
  double minimum_distance_;
  double reserve_percent_;
  double safety_factor_;
  double alert_repeat_period_;
  std::optional<double> battery_percent_;
  std::optional<double> battery_received_;
  std::optional<Pose> pose_;
  std::optional<double> odometry_received_;
  std::optional<mavros_msgs::msg::State> vehicle_state_;
  std::optional<double> state_received_;
  std::optional<Pose> home_pose_;
  std::optional<Pose> ground_pose_;
  bool home_locked_ = false;
  std::optional<Pose> mavros_home_;
  std::optional<Baseline> baseline_;
  bool phase_active_ = false;
  bool interval_invalid_ = false;
  std::optional<Pose> last_good_pose_;
  std::optional<double> last_good_percent_;
  double last_alert_ = -std::numeric_limits<double>::infinity();
  bool alert_active_ = false;
  std::string unavailable_;
  std::optional<Segment> active_;
  // A dictionary keyed by sample ID; poses and strings are independent values.
  std::map<uint64_t, Record> records_;
  uint64_t next_sample_id_ = 1;
  rclcpp::Publisher<Usage>::SharedPtr usage_publisher_;
  rclcpp::Publisher<Estimate>::SharedPtr estimate_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kv_executive

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // Default mutually exclusive callbacks and a single-threaded executor keep
    // all telemetry, model records and publishing lifetimes serial.
    rclcpp::spin(std::make_shared<kv_executive::BatteryMonitor>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("battery_monitor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
