#include <algorithm>
#include <chrono>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <utility>
#include <vector>
#include <set>

#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <tf2/utils.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include "kv_executive/srv/plan_path.hpp"
#include "kv_executive/srv/plan_to_pose.hpp"
#include "kv_executive/srv/plan_inspection_path.hpp"
#include "kv_executive/srv/query_inspection_geometry.hpp"
#include "kv_executive/msg/fire_source_report.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class InspectionMotionValidator : public ob::MotionValidator
{
public:
  InspectionMotionValidator(
    const ob::SpaceInformationPtr & information,
    std::function<bool(const ob::State *, const ob::State *)> check)
  : ob::MotionValidator(information), check_(std::move(check)) {}

  bool checkMotion(const ob::State * start, const ob::State * end) const override
  {
    return check_(start, end);
  }

  bool checkMotion(
    const ob::State * start, const ob::State * end,
    std::pair<ob::State *, double> & last_valid) const override
  {
    if (check_(start, end)) {
      return true;
    }
    // A conservative valid prefix is sufficient for RRTConnect; never claim
    // a partially checked segment is free.
    last_valid.second = 0.0;
    if (last_valid.first) {
      si_->copyState(last_valid.first, start);
    }
    return false;
  }

private:
  std::function<bool(const ob::State *, const ob::State *)> check_;
};

class MotionPlanner : public rclcpp::Node
{
public:
  MotionPlanner()
  : Node("motion_planner")
  {
    loadPlannerParameters();
    RCLCPP_INFO(
      this->get_logger(),
      "Inspection collision policy: %s; source timeout=%.3f ROS s; wall watchdog=%.3f s "
      "(map reception in real time, clock progress in simulation)",
      inspection_require_observed_free_space_ ? "require observed free air" :
      "permit unknown air; current and recent occupied voxels remain blocking",
      inspection_map_timeout_, inspection_map_timeout_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_, rclcpp::SensorDataQoS(),
            std::bind(&MotionPlanner::odometryCallback, this, std::placeholders::_1));

    if (!home_from_first_odometry_) {
      home_pos_sub_ = this->create_subscription<mavros_msgs::msg::HomePosition>(
                home_position_topic_, rclcpp::SensorDataQoS(),
                std::bind(&MotionPlanner::homePositionCallback, this, std::placeholders::_1));
    }

    param_mavros_ = this->create_client<rcl_interfaces::srv::GetParameters>(
            mavros_parameter_service_);
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
      std::bind(&MotionPlanner::requestMaximumAltitude, this));

    octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&MotionPlanner::octomapCallback, this, std::placeholders::_1)
    );
    octomap_watchdog_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MotionPlanner::checkOctomapReception, this));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "planned_path", rclcpp::QoS(1).reliable().transient_local());
    planning_service_ = this->create_service<kv_executive::srv::PlanPath>(
            "plan_path",
            std::bind(
              &MotionPlanner::planPathCallback, this,
              std::placeholders::_1, std::placeholders::_2));
    target_planning_service_ = this->create_service<kv_executive::srv::PlanToPose>(
            "plan_to_pose",
            std::bind(
              &MotionPlanner::planToPoseCallback, this,
              std::placeholders::_1, std::placeholders::_2));
    inspection_planning_service_ = this->create_service<kv_executive::srv::PlanInspectionPath>(
      "plan_inspection_path", std::bind(&MotionPlanner::planInspectionPathCallback, this,
      std::placeholders::_1, std::placeholders::_2));
    inspection_geometry_service_ =
      this->create_service<kv_executive::srv::QueryInspectionGeometry>(
      "query_inspection_geometry",
      std::bind(&MotionPlanner::queryInspectionGeometry, this,
      std::placeholders::_1, std::placeholders::_2));
    fire_sources_subscription_ = this->create_subscription<kv_executive::msg::FireSourceReport>(
            "fire_sources", rclcpp::QoS(100).reliable().transient_local(),
            std::bind(&MotionPlanner::fireSourceCallback, this, std::placeholders::_1));

    createStateSpace();
  }

  void createPlanningSetup(
    const ob::StateValidityCheckerFn & validity = {},
    const std::function<bool(const ob::State *, const ob::State *)> & motion = {})
  {
    ss_ = std::make_shared<og::SimpleSetup>(space_);
    ss_->setStateValidityChecker(validity ? validity :
      ob::StateValidityCheckerFn([this](const ob::State * state) {return isStateValid(state);}));
    const auto si = ss_->getSpaceInformation();
    si->setStateValidityCheckingResolution(validity_resolution_fraction_);
    if (motion) {
      si->setMotionValidator(std::make_shared<InspectionMotionValidator>(si, motion));
    }
    auto planner = std::make_shared<og::RRTConnect>(si);
    ss_->setPlanner(planner);
  }

  bool planPath(nav_msgs::msg::Path & path_message)
  {
    last_planning_error_.clear();
    if (!updatePlanningPoses()) {
      return false;
    }

    geometry_msgs::msg::Point goal_position;
    if (!randomWalkGoal(goal_position)) {
      return false;
    }
    return createPath(goal_position, path_message);
  }

  bool randomWalkGoal(geometry_msgs::msg::Point & goal_position)
  {
    updateReportedSiteRegions();
    const double min_x = current_pose.position.x - horizontal_range_;
    const double max_x = current_pose.position.x + horizontal_range_;
    const double min_y = current_pose.position.y - horizontal_range_;
    const double max_y = current_pose.position.y + horizontal_range_;
    const double min_z = std::max(
            current_pose.position.z - vertical_range_,
            home_pose.z + minimum_altitude_ + bound_margin_);
    const double max_z = std::min(
            current_pose.position.z + vertical_range_,
            home_pose.z + maximum_altitude_ - bound_margin_);

    for (std::size_t attempt = 0; attempt < 128; ++attempt) {
      goal_position.x =
        std::uniform_real_distribution<double>(min_x, max_x)(random_generator_);
      goal_position.y =
        std::uniform_real_distribution<double>(min_y, max_y)(random_generator_);

      // updatePositionSpaceBounds() reports an invalid altitude range.
      goal_position.z = min_z <= max_z ?
        std::uniform_real_distribution<double>(min_z, max_z)(random_generator_) :
        current_pose.position.z;
      if (outsideReportedSites(goal_position)) {
        return true;
      }
    }
    return failPlanning(
      "No exploration goal outside reported fire-source regions after 128 attempts");
  }

  bool createPath(
    const geometry_msgs::msg::Point & goal_position,
    nav_msgs::msg::Path & path_message,
    const geometry_msgs::msg::Quaternion * goal_orientation = nullptr,
    const ob::StateValidityCheckerFn & validity = {},
    const std::function<bool(const ob::State *, const ob::State *)> & motion = {},
    const geometry_msgs::msg::Pose * start_override = nullptr, bool publish = true)
  {
    if (!current_pose_received_ || !home_pose_received_ || !occupancy_map_received_) {
      return failPlanning("Required odometry, home, or OctoMap data became unavailable");
    }

    if (!updatePositionSpaceBounds(goal_position, goal_orientation != nullptr, start_override)) {
      return false;
    }

    createPlanningSetup(validity, motion);
    const auto & initial_pose = start_override ? *start_override : current_pose;
        // create a start state
    ob::ScopedState<> start(space_);
    auto *compound_state = start->as<ob::CompoundStateSpace::StateType>();
    auto *position_state = compound_state->as<ob::RealVectorStateSpace::StateType>(0);
    position_state->values[0] = initial_pose.position.x;
    position_state->values[1] = initial_pose.position.y;
    position_state->values[2] = initial_pose.position.z;
    auto *yaw_state = compound_state->as<ob::SO2StateSpace::StateType>(1);
    yaw_state->value = tf2::getYaw(initial_pose.orientation);

    ob::ScopedState<> goal(space_);
    auto *goal_compound_state = goal->as<ob::CompoundStateSpace::StateType>();
    auto *goal_position_state =
      goal_compound_state->as<ob::RealVectorStateSpace::StateType>(0);
    goal_position_state->values[0] = goal_position.x;
    goal_position_state->values[1] = goal_position.y;
    goal_position_state->values[2] = goal_position.z;
    auto *goal_yaw_state = goal_compound_state->as<ob::SO2StateSpace::StateType>(1);
    goal_yaw_state->value = tf2::getYaw(
      goal_orientation ? *goal_orientation : current_pose.orientation);

    const auto space_information = ss_->getSpaceInformation();
    if (!space_information->isValid(start.get())) {
      return failPlanning(
        "OMPL start state is invalid: check altitude bounds, home reference, and obstacle clearance");
    }
    if (!space_information->isValid(goal.get())) {
      return failPlanning(
        "OMPL goal state is invalid: check altitude bounds and obstacle clearance");
    }
    ss_->setStartAndGoalStates(start, goal);

    const auto solved = ss_->solve(planning_timeout_);
    if (!solved) {
      return failPlanning("No OMPL solution found within the planning timeout");
    }
    if (solved != ob::PlannerStatus::EXACT_SOLUTION) {
      return failPlanning(
        "OMPL returned an approximate solution; requested endpoint was not reached");
    }
    ss_->simplifySolution(simplification_time_);
    auto & solution_path = ss_->getSolutionPath();

    path_message = processSolutionPath(solution_path, goal_orientation, publish);

    return true;
  }

  nav_msgs::msg::Path processSolutionPath(
    const og::PathGeometric & solution_path,
    const geometry_msgs::msg::Quaternion * goal_orientation = nullptr, bool publish = true)
  {
    nav_msgs::msg::Path path_message;
    path_message.header.stamp = this->now();
    {
      std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      path_message.header.frame_id = occupancy_map_frame_;
    }

    if (path_message.header.frame_id.empty()) {
      RCLCPP_WARN(
                this->get_logger(),
                "Cannot publish the planned path because the OctoMap frame is empty");
    }

    path_message.poses.reserve(solution_path.getStateCount());
    for (std::size_t index = 0; index < solution_path.getStateCount(); ++index) {
      const auto *compound_state =
        solution_path.getState(index)->as<ob::CompoundStateSpace::StateType>();
      const auto *position_state =
        compound_state->as<ob::RealVectorStateSpace::StateType>(0);
      const auto *yaw_state =
        compound_state->as<ob::SO2StateSpace::StateType>(1);

      geometry_msgs::msg::Pose pose;
      pose.position.x = position_state->values[0];
      pose.position.y = position_state->values[1];
      pose.position.z = position_state->values[2];
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = std::sin(yaw_state->value * 0.5);
      pose.orientation.w = std::cos(yaw_state->value * 0.5);
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_message.header;
      pose_stamped.pose = pose;
      path_message.poses.push_back(std::move(pose_stamped));
    }
    if (goal_orientation && !path_message.poses.empty()) {
      path_message.poses.back().pose.orientation = *goal_orientation;
    }

    if (publish && !path_message.header.frame_id.empty()) {
      path_publisher_->publish(path_message);
      RCLCPP_INFO(
                this->get_logger(), "Published planned path with %zu poses in frame '%s'",
                path_message.poses.size(), path_message.header.frame_id.c_str());
    }

    return path_message;
  }

private:
  struct InspectionMap
  {
    std::shared_ptr<const octomap::OcTree> tree;
    std_msgs::msg::Header header;
    double received{0.0};
  };

  struct SupportSurface
  {
    bool valid{false};
    geometry_msgs::msg::Point point;
    double uncertainty{0.0};
  };

  static double stampSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  std::string inspectionSourceIssue(const std_msgs::msg::Header & header) const
  {
    const double source = stampSeconds(header.stamp);
    const double now = this->get_clock()->now().seconds();
    if (header.frame_id.empty()) {
      return "missing map frame_id";
    }
    if (source <= 0.0) {
      return "zero or invalid source stamp";
    }
    if (now <= 0.0) {
      return "ROS clock is not initialized";
    }
    if (now - source < -inspection_future_tolerance_) {
      return "source stamp is in the future; check use_sim_time and clock alignment";
    }
    if (now - source > inspection_map_timeout_) {
      return "source observation is too old; check mapping processing and queue delay";
    }
    return {};
  }

  std::string mapAgeDetail(const InspectionMap & map) const
  {
    std::ostringstream detail;
    detail.precision(12);
    const double source = stampSeconds(map.header.stamp);
    const double now = this->get_clock()->now().seconds();
    detail << "; source_age=";
    if (source > 0.0 && now > 0.0) {
      detail << now - source << " s";
    } else {
      detail << "unavailable";
    }
    detail << "; receipt_silence=" << octomap_clock_.now().seconds() - map.received <<
      " s; timeout=" << inspection_map_timeout_ << " s; source_stamp=" << source <<
      "; ros_now=" << now;
    return detail.str();
  }

  std::string inspectionMapIssue(const InspectionMap & map) const
  {
    const double ros_now = this->get_clock()->now().seconds();
    const double wall_now = octomap_clock_.now().seconds();
    if (ros_now != inspection_last_ros_clock_) {
      inspection_last_ros_clock_ = ros_now;
      inspection_last_clock_progress_ = wall_now;
    }
    std::string cause = map.tree ? inspectionSourceIssue(map.header) : "no accepted OctoMap";
    if (cause.empty()) {
      if (this->get_parameter("use_sim_time").as_bool()) {
        // Sensor stamps and vehicle motion share simulated time. A slow but
        // advancing simulation must not age a map faster than those inputs.
        if (wall_now - inspection_last_clock_progress_ > inspection_map_timeout_) {
          cause = "simulation clock has stopped advancing";
        }
      } else {
        const double silence = wall_now - map.received;
        if (silence < 0.0 || silence > inspection_map_timeout_) {
          cause = "accepted map reception is too old; check map output rate and planner load";
        }
      }
    }
    return cause.empty() ? std::string{} : cause + mapAgeDetail(map);
  }

  bool inspectionMapFresh(const InspectionMap & map) const
  {
    return inspectionMapIssue(map).empty();
  }

  static bool finitePoint(const geometry_msgs::msg::Point & point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }

  geometry_msgs::msg::Point inspectionPoint(
    const geometry_msgs::msg::PointStamped & input, const std::string & frame)
  {
    if (input.header.frame_id.empty() || !finitePoint(input.point)) {
      throw std::invalid_argument("Inspection point requires a frame and finite coordinates");
    }
    geometry_msgs::msg::PointStamped output;
    tf_buffer_->transform(input, output, frame, tf2::durationFromSec(transform_timeout_));
    if (!finitePoint(output.point)) {
      throw std::invalid_argument("Transformed inspection point is invalid");
    }
    return output.point;
  }

  geometry_msgs::msg::Vector3 inspectionAxis(
    const geometry_msgs::msg::Vector3Stamped & input, const std::string & frame)
  {
    geometry_msgs::msg::Vector3Stamped output;
    if (input.header.frame_id.empty()) {
      output.vector.z = 1.0;
    } else {
      const auto & v = input.vector;
      if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
        std::hypot(std::hypot(v.x, v.y), v.z) < 1e-9)
      {
        throw std::invalid_argument("Inspection keepout axis must be finite and nonzero");
      }
      tf_buffer_->transform(input, output, frame, tf2::durationFromSec(transform_timeout_));
    }
    auto & v = output.vector;
    const double norm = std::hypot(std::hypot(v.x, v.y), v.z);
    if (!std::isfinite(norm) || norm < 1e-9) {
      throw std::invalid_argument("Transformed inspection keepout axis is invalid");
    }
    v.x /= norm;
    v.y /= norm;
    v.z /= norm;
    return v;
  }

  static double planeRadius(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & center,
    const geometry_msgs::msg::Vector3 & axis)
  {
    const double x = point.x - center.x, y = point.y - center.y, z = point.z - center.z;
    const double along = x * axis.x + y * axis.y + z * axis.z;
    return std::hypot(std::hypot(x - along * axis.x, y - along * axis.y),
      z - along * axis.z);
  }

  static double segmentPlaneRadius(
    const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end,
    const geometry_msgs::msg::Point & center, const geometry_msgs::msg::Vector3 & axis)
  {
    const std::array<double, 3> up{axis.x, axis.y, axis.z};
    std::array<double, 3> a{start.x - center.x, start.y - center.y, start.z - center.z};
    std::array<double, 3> delta{end.x - start.x, end.y - start.y, end.z - start.z};
    double a_up = 0.0, delta_up = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      a_up += a[i] * up[i];
      delta_up += delta[i] * up[i];
    }
    double dot = 0.0, length = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      a[i] -= a_up * up[i];
      delta[i] -= delta_up * up[i];
      dot += a[i] * delta[i];
      length += delta[i] * delta[i];
    }
    const double fraction = length > 0.0 ? std::clamp(-dot / length, 0.0, 1.0) : 0.0;
    return std::hypot(std::hypot(a[0] + fraction * delta[0],
      a[1] + fraction * delta[1]), a[2] + fraction * delta[2]);
  }

  static double leafSizeAt(const octomap::OcTree & tree, const octomap::point3d & point)
  {
    if (!tree.nodeHasChildren(tree.getRoot())) {
      return tree.getNodeSize(0);
    }
    for (unsigned depth = 1; depth <= tree.getTreeDepth(); ++depth) {
      const auto * node = tree.search(point, depth);
      if (!node) {
        return tree.getResolution();
      }
      if (!tree.nodeHasChildren(node)) {
        return tree.getNodeSize(depth);
      }
    }
    return tree.getResolution();
  }

  static void occupancyEvidence(
    const std::vector<InspectionMap> & history, const octomap::point3d & point,
    uint32_t & occupied, uint32_t & free)
  {
    occupied = free = 0;
    for (const auto & map : history) {
      const auto * node = map.tree->search(point);
      if (node) {
        if (map.tree->isNodeOccupied(node)) {
          ++occupied;
        } else {
          ++free;
        }
      }
    }
  }

  SupportSurface inspectionSupport(
    const InspectionMap & latest, const std::vector<InspectionMap> & history,
    const geometry_msgs::msg::Point & query) const
  {
    SupportSurface result;
    const auto & tree = *latest.tree;
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    double uncertainty = 0.0;
    const std::array<double, 3> offsets{-inspection_ground_patch_radius_, 0.0,
      inspection_ground_patch_radius_};
    for (const double dx : offsets) {
      for (const double dy : offsets) {
        const octomap::point3d origin(query.x + dx, query.y + dy, query.z);
        octomap::point3d hit;
        if (!tree.castRay(origin, octomap::point3d(0.0, 0.0, -1.0), hit,
          !inspection_require_observed_free_space_, inspection_max_range_))
        {
          return result;  // An actual observed occupied surface is always required.
        }
        uint32_t occupied, free;
        occupancyEvidence(history, hit, occupied, free);
        const double size = leafSizeAt(tree, hit);
        if (occupied < 2 || free != 0 || size > inspection_ground_patch_radius_ ||
          hit.z() >= query.z - tree.getResolution() * 0.5)
        {
          return result;
        }
        low = std::min(low, static_cast<double>(hit.z()));
        high = std::max(high, static_cast<double>(hit.z()));
        uncertainty = std::max(uncertainty, size * 0.5);
      }
    }
    if (high - low > std::max(0.2, tree.getResolution())) {
      return result;  // A vertical wall or steep/irregular patch is not support.
    }
    result.valid = true;
    result.point = query;
    result.point.z = high;
    result.uncertainty = uncertainty + high - low;
    return result;
  }

  SupportSurface inspectionTerrainEnvelope(
    const InspectionMap & latest, const std::vector<InspectionMap> & history,
    const geometry_msgs::msg::Point & query, bool estimate_only = false) const
  {
    SupportSurface result;
    const auto & tree = *latest.tree;
    const double radius = std::max(inspection_ground_patch_radius_,
      drone_radius_ + obstacle_clearance_);
    if (!inspection_require_observed_free_space_) {
      // A lidar footprint need not contain a ground return in every column.
      // Use actual occupied leaves, including recent ones, to bound known
      // terrain. Missing returns are not a measured ground surface.
      const octomap::point3d lower(query.x - radius, query.y - radius,
        query.z - inspection_max_range_);
      const octomap::point3d upper(query.x + radius, query.y + radius, query.z);
      double highest = -std::numeric_limits<double>::infinity();
      double uncertainty = 0.0;
      const auto observe = [&](const InspectionMap & map) {
          octomap::OcTreeKey low_key, high_key;
          if (!map.tree->coordToKeyChecked(lower, low_key) ||
            !map.tree->coordToKeyChecked(upper, high_key))
          {
            throw std::invalid_argument("Terrain bounds exceed the OctoMap key index range");
          }
          for (auto leaf = map.tree->begin_leafs_bbx(low_key, high_key),
            end = map.tree->end_leafs_bbx(); leaf != end; ++leaf)
          {
            if (!map.tree->isNodeOccupied(*leaf)) {
              continue;
            }
            const double half = leaf.getSize() * 0.5;
            const double dx = std::max(0.0, std::abs(leaf.getX() - query.x) - half);
            const double dy = std::max(0.0, std::abs(leaf.getY() - query.y) - half);
            if (dx * dx + dy * dy > radius * radius || leaf.getZ() - half > query.z) {
              continue;
            }
            highest = std::max(highest, leaf.getZ() + half);
            uncertainty = std::max(uncertainty, half);
          }
        };
      observe(latest);
      for (const auto & previous : history) {
        if (previous.tree != latest.tree) {
          observe(previous);
        }
      }
      if (std::isfinite(highest)) {
        result.valid = true;
        result.point = query;
        result.point.z = highest;
        result.uncertainty = uncertainty;
      }
      return result;
    }
    octomap::OcTreeKey low, high;
    if (!tree.coordToKeyChecked(query.x - radius, query.y - radius, query.z, low) ||
      !tree.coordToKeyChecked(query.x + radius, query.y + radius, query.z, high) ||
      static_cast<double>(high[0] - low[0] + 1) * (high[1] - low[1] + 1) > 4096.0)
    {
      return result;
    }
    const double half = tree.getResolution() * 0.5;
    double highest = -std::numeric_limits<double>::infinity();
    double uncertainty = half;
    for (unsigned x = low[0]; x <= high[0]; ++x) {
      for (unsigned y = low[1]; y <= high[1]; ++y) {
        const auto cell = tree.keyToCoord(octomap::OcTreeKey(x, y, low[2]));
        const double dx = std::max(0.0, std::abs(cell.x() - query.x) - half);
        const double dy = std::max(0.0, std::abs(cell.y() - query.y) - half);
        if (dx * dx + dy * dy > radius * radius) {
          continue;
        }
        octomap::point3d hit;
        const octomap::point3d origin(cell.x(), cell.y(), query.z);
        // Sparse lidar air can be unknown while a real terrain hit below is
        // observed. Strict mode additionally requires the full column to be free.
        if (!tree.castRay(origin, octomap::point3d(0.0, 0.0, -1.0), hit,
          estimate_only || !inspection_require_observed_free_space_, inspection_max_range_))
        {
          return result;
        }
        uint32_t occupied, free;
        occupancyEvidence(history, hit, occupied, free);
        const double size = leafSizeAt(tree, hit);
        const double top = (std::floor(static_cast<double>(hit.z()) / size) + 1.0) * size;
        if (occupied < 2 || free != 0 || top >= query.z - tree.getResolution() * 0.5) {
          return result;
        }
        // The highest voxel top is conservative on slopes and rough patches.
        // Its spread is terrain geometry, not uncertainty in a flat plane.
        highest = std::max(highest, top);
        uncertainty = std::max(uncertainty, size * 0.5);
      }
    }
    if (!std::isfinite(highest)) {
      return result;
    }
    result.valid = true;
    result.point = query;
    result.point.z = highest;
    result.uncertainty = uncertainty;
    return result;
  }

  double inspectionGroundClearance(
    const InspectionMap & latest, const std::vector<InspectionMap> & history,
    const geometry_msgs::msg::Point & point, bool terrain_envelope) const
  {
    const auto support = terrain_envelope ?
      inspectionTerrainEnvelope(latest, history, point) :
      inspectionSupport(latest, history, point);
    if (support.valid) {
      return point.z - support.point.z - support.uncertainty;
    }
    // This is a lower bound within the checked vertical range, not a measured
    // ground height. Strict mode still requires observed support below the path.
    return terrain_envelope && !inspection_require_observed_free_space_ ?
           std::max(0.0, inspection_max_range_ - latest.tree->getResolution() * 0.5) :
           std::numeric_limits<double>::quiet_NaN();
  }

  bool inspectionBodyFree(
    const InspectionMap & latest, const std::vector<InspectionMap> & history,
    const geometry_msgs::msg::Point & point, std::string & reason) const
  {
    const auto & tree = *latest.tree;
    const double radius = drone_radius_ + obstacle_clearance_;
    const auto reject = [&](const std::string & cause,
      const octomap::point3d * voxel = nullptr) {
        std::ostringstream detail;
        detail.precision(12);
        detail << cause << "; path sample=(" << point.x << ", " << point.y << ", " <<
          point.z << "), clearance radius=" << radius << " m";
        if (voxel) {
          detail << ", blocking voxel=(" << voxel->x() << ", " << voxel->y() << ", " <<
            voxel->z() << ")";
        }
        reason = detail.str();
        return false;
      };
    octomap::OcTreeKey low, high;
    if (!tree.coordToKeyChecked(point.x - radius, point.y - radius, point.z - radius, low) ||
      !tree.coordToKeyChecked(point.x + radius, point.y + radius, point.z + radius, high))
    {
      return reject("body clearance bounds exceed the OctoMap key index range");
    }
    const double half = tree.getResolution() * 0.5;
    const double cells = static_cast<double>(high[0] - low[0] + 1) *
      (high[1] - low[1] + 1) * (high[2] - low[2] + 1);
    if (cells > 50000.0) {
      std::ostringstream detail;
      detail << "body clearance query exceeds the voxel work limit: cells=" << cells <<
        ", limit=50000, resolution=" << tree.getResolution() << " m";
      return reject(detail.str());  // Bound work for unusually fine-resolution maps.
    }
    for (unsigned x = low[0]; x <= high[0]; ++x) {
      for (unsigned y = low[1]; y <= high[1]; ++y) {
        for (unsigned z = low[2]; z <= high[2]; ++z) {
          const auto center = tree.keyToCoord(octomap::OcTreeKey(x, y, z));
          const double dx = std::max(0.0, std::abs(center.x() - point.x) - half);
          const double dy = std::max(0.0, std::abs(center.y() - point.y) - half);
          const double dz = std::max(0.0, std::abs(center.z() - point.z) - half);
          if (dx * dx + dy * dy + dz * dz > radius * radius) {
            continue;
          }
          const auto * node = tree.search(center);
          if (!node && inspection_require_observed_free_space_) {
            return reject("unknown cell in the latest map", &center);
          }
          if (node && tree.isNodeOccupied(node)) {
            return reject("occupied cell in the latest map", &center);
          }
          // Recent occupied observations remain blocking even when the latest
          // map flickers free. Temporal inconsistency is only diagnostic.
          for (const auto & previous : history) {
            const auto * old_node = previous.tree->search(center);
            if (old_node && previous.tree->isNodeOccupied(old_node)) {
              return reject("occupied cell in recent map history", &center);
            }
          }
        }
      }
    }
    return true;
  }

  bool inspectionCorridor(
    const InspectionMap & latest, const std::vector<InspectionMap> & history,
    const std::vector<geometry_msgs::msg::Point> & vertices,
    const geometry_msgs::msg::Point & center, double minimum_radius, double minimum_agl,
    std::string & reason, const geometry_msgs::msg::Vector3 & axis,
    bool allow_initial_climb = false, double tracking_tolerance = 0.0,
    bool terrain_envelope = false, bool allow_terrain_escape = false,
    const geometry_msgs::msg::Point * escape_origin = nullptr,
    bool terrain_escape_complete = false, bool require_final_clearance = true) const
  {
    const double spacing = std::min(0.2, latest.tree->getResolution() * 0.5);
    std::size_t samples = 0;
    bool escape_prefix = (allow_initial_climb || allow_terrain_escape) && minimum_agl > 0.0 &&
      !terrain_escape_complete;
    bool reached_clearance = minimum_agl <= 0.0 || terrain_escape_complete;
    double previous_height = escape_origin && escape_prefix ?
      0.0 : -std::numeric_limits<double>::infinity();
    double previous_agl = -std::numeric_limits<double>::infinity();
    const auto & origin = escape_origin ? *escape_origin : vertices.front();
    if (escape_origin && escape_prefix) {
      previous_agl = inspectionGroundClearance(latest, history, origin, terrain_envelope);
      if (std::isnan(previous_agl)) {
        reason = "Inspection original escape anchor has unverified terrain clearance";
        return false;
      }
    }
    for (std::size_t segment = 0; segment < vertices.size(); ++segment) {
      const auto & start = vertices[segment ? segment - 1 : 0];
      const auto & end = vertices[segment];
      const double length = std::hypot(std::hypot(end.x - start.x, end.y - start.y),
          end.z - start.z);
      if (!std::isfinite(length) || length / spacing > 10000.0) {
        throw std::invalid_argument("Inspection corridor exceeds the sampling limit");
      }
      const auto count = std::max<std::size_t>(1, std::ceil(length / spacing));
      const double closest_radius = segmentPlaneRadius(start, end, center, axis);
      if (minimum_radius > 0.0 && closest_radius + 1e-6 < minimum_radius) {
        std::ostringstream detail;
        detail.precision(12);
        detail << "Inspection corridor violates the requested standoff radius: actual=" <<
          closest_radius << " m, requested=" << minimum_radius << " m, point=(" <<
          end.x << ", " << end.y << ", " << end.z << ")";
        reason = detail.str();
        return false;
      }
      for (std::size_t index = 0; index <= count; ++index) {
        if (++samples > 10000) {
          throw std::invalid_argument("Inspection corridor exceeds the sampling limit");
        }
        const double fraction = static_cast<double>(index) / count;
        geometry_msgs::msg::Point point;
        point.x = start.x + fraction * (end.x - start.x);
        point.y = start.y + fraction * (end.y - start.y);
        point.z = start.z + fraction * (end.z - start.z);
        const double actual_radius = planeRadius(point, center, axis);
        // TF round trips and an analytically constructed boundary endpoint can
        // differ at floating-point precision. This tolerance applies only to
        // the radius comparison; occupancy and terrain checks stay unchanged.
        if (minimum_radius > 0.0 && actual_radius + 1e-6 < minimum_radius) {
          std::ostringstream detail;
          detail.precision(12);
          detail << "Inspection corridor violates the requested standoff radius: actual=" <<
            actual_radius << " m, requested=" << minimum_radius << " m, point=(" <<
            point.x << ", " << point.y << ", " << point.z << ")";
          reason = detail.str();
          return false;
        }
        std::string body_reason;
        if (!inspectionBodyFree(latest, history, point, body_reason)) {
          reason = "Inspection corridor contains occupied or unknown body clearance: " +
            body_reason;
          return false;
        }
        if (minimum_agl > 0.0) {
          const double agl = inspectionGroundClearance(latest, history, point, terrain_envelope);
          if (std::isnan(agl)) {
            reason = "Inspection corridor has unknown or insufficient support-surface clearance";
            return false;
          }
          const double height = (point.x - origin.x) * axis.x +
            (point.y - origin.y) * axis.y + (point.z - origin.z) * axis.z;
          if (agl + 1e-6 < minimum_agl) {
            if (reached_clearance) {
              reason = "Inspection corridor drops below required support-surface clearance "
                "after the initial escape completed";
              return false;
            }
            if (!escape_prefix || reached_clearance || axis.z <= 1e-6 ||
              planeRadius(point, origin, axis) >
              (allow_terrain_escape ? 2.0 : 0.0) + tracking_tolerance + 1e-6 ||
              height + tracking_tolerance + 1e-6 < previous_height ||
              agl + tracking_tolerance + 1e-6 < previous_agl)
            {
              reason = allow_terrain_escape ?
                "Inspection terrain escape must stay within 2 m horizontal radius and "
                "never decrease height or measured clearance before reaching minimum_agl" :
                "Inspection corridor has unknown or insufficient support-surface clearance; "
                "only a fixed-horizontal-position upward initial climb may escape low clearance";
              std::ostringstream detail;
              detail.precision(12);
              detail << reason << "; sample=(" << point.x << ", " << point.y << ", " <<
                point.z << "), horizontal_radius=" << planeRadius(point, origin, axis) <<
                " m, height=" << height << " m, previous_height=" << previous_height <<
                " m, clearance=" << agl << " m, previous_clearance=" << previous_agl <<
                " m, required_clearance=" << minimum_agl << " m";
              reason = detail.str();
              return false;
            }
          } else {
            reached_clearance = true;
            escape_prefix = false;
          }
          previous_height = std::max(previous_height, height);
          previous_agl = std::max(previous_agl, agl);
        }
      }
    }
    if (require_final_clearance && !reached_clearance) {
      reason = "Inspection initial climb does not reach the requested support-surface clearance";
      return false;
    }
    reason = inspection_require_observed_free_space_ ?
      "Inspection corridor is observed free with the requested clearances" :
      "Inspection corridor avoids current and recent occupied voxels with the requested clearances";
    return true;
  }

  bool inspectionClearanceEscape(
    const InspectionMap & latest, const std::vector<InspectionMap> & history,
    const geometry_msgs::msg::Point & origin, const geometry_msgs::msg::Point & center,
    double minimum_radius, double minimum_agl, const geometry_msgs::msg::Vector3 & axis,
    double ceiling, std::vector<geometry_msgs::msg::Point> & prefix,
    std::string & reason) const
  {
    // The ordinary OMPL state checker requires minimum_agl at every state.
    // Search the exceptional low-clearance prefix separately, with the same
    // fixed anchor and monotonic swept-corridor checks used in flight.
    const auto support = inspectionTerrainEnvelope(latest, history, origin);
    if (!support.valid) {
      reason = "Initial terrain envelope is unavailable";
      return false;
    }
    const double initial_agl = origin.z - support.point.z - support.uncertainty;
    const double horizontal_norm = std::hypot(axis.z, axis.x);
    const std::array<double, 3> right{axis.z / horizontal_norm, 0.0,
      -axis.x / horizontal_norm};
    const std::array<double, 3> side{
      axis.y * right[2] - axis.z * right[1],
      axis.z * right[0] - axis.x * right[2],
      axis.x * right[1] - axis.y * right[0]};
    struct Vertex
    {
      std::array<int, 3> cell;
      geometry_msgs::msg::Point point;
      double agl, cost;
      std::size_t parent;
    };
    const auto no_parent = std::numeric_limits<std::size_t>::max();
    std::vector<Vertex> vertices{{{0, 0, 0}, origin, initial_agl, 0.0, no_parent}};
    std::vector<std::size_t> frontier{0};
    std::map<std::array<int, 3>, double> best_cost{{{0, 0, 0}, 0.0}};
    constexpr double step = 0.5;
    constexpr std::size_t maximum_expansions = 384;
    const double maximum_rise = std::min(20.0, std::max(2.0, minimum_agl * 2.0));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(
      std::max(0.05, planning_timeout_));
    std::size_t expanded = 0;
    const auto score = [&](std::size_t index) {
        const auto & vertex = vertices[index];
        return vertex.cost + std::max(0.0, minimum_agl - vertex.agl);
      };
    while (!frontier.empty() && expanded < maximum_expansions &&
      std::chrono::steady_clock::now() < deadline)
    {
      const auto selected = std::min_element(frontier.begin(), frontier.end(),
          [&](std::size_t a, std::size_t b) {return score(a) < score(b);});
      const std::size_t parent = *selected;
      frontier.erase(selected);
      const auto current = vertices[parent];
      if (current.cost > best_cost[current.cell] + 1e-6) {
        continue;
      }
      ++expanded;
      auto raised = current.point;
      const double rise = std::max(0.0, (minimum_agl - current.agl + 1e-4) / axis.z);
      raised.x += rise * axis.x;
      raised.y += rise * axis.y;
      raised.z += rise * axis.z;
      if (raised.z <= ceiling && inspectionCorridor(latest, history,
        {current.point, raised}, center, minimum_radius, minimum_agl, reason, axis,
        true, 0.0, true, true, &origin))
      {
        prefix.clear();
        for (auto index = parent; index != no_parent; index = vertices[index].parent) {
          prefix.push_back(vertices[index].point);
        }
        std::reverse(prefix.begin(), prefix.end());
        if (rise > 1e-9) {
          prefix.push_back(raised);
        }
        return true;
      }
      for (int up = 1; up >= 0; --up) {
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (up == 0 && dx == 0 && dy == 0) {
              continue;
            }
            const std::array<int, 3> cell{current.cell[0] + dx,
              current.cell[1] + dy, current.cell[2] + up};
            if (cell[0] * cell[0] + cell[1] * cell[1] > 16 ||
              cell[2] * step > maximum_rise)
            {
              continue;
            }
            const double cost = current.cost + step * std::sqrt(dx * dx + dy * dy + up * up);
            const auto previous = best_cost.find(cell);
            if (previous != best_cost.end() && previous->second <= cost + 1e-6) {
              continue;
            }
            auto point = origin;
            point.x += step * (cell[0] * right[0] + cell[1] * side[0] + cell[2] * axis.x);
            point.y += step * (cell[0] * right[1] + cell[1] * side[1] + cell[2] * axis.y);
            point.z += step * (cell[0] * right[2] + cell[1] * side[2] + cell[2] * axis.z);
            if (point.z > ceiling) {
              continue;
            }
            std::string edge_reason;
            if (!inspectionCorridor(latest, history,
              {current.point, point}, center, minimum_radius, minimum_agl, edge_reason, axis,
              true, 0.0, true, true, &origin, false, false))
            {
              reason = edge_reason;
              continue;
            }
            const auto terrain = inspectionTerrainEnvelope(latest, history, point);
            if (!terrain.valid) {
              continue;
            }
            best_cost[cell] = cost;
            vertices.push_back({cell, point, point.z - terrain.point.z - terrain.uncertainty,
                cost, parent});
            frontier.push_back(vertices.size() - 1);
          }
        }
      }
    }
    reason = "Bounded multi-segment escape exhausted after " + std::to_string(expanded) +
      " states; last rejection: " + reason;
    return false;
  }

  void queryInspectionGeometry(
    const kv_executive::srv::QueryInspectionGeometry::Request::SharedPtr request,
    kv_executive::srv::QueryInspectionGeometry::Response::SharedPtr response)
  {
    using Response = kv_executive::srv::QueryInspectionGeometry::Response;
    response->success = false;
    response->error_code = Response::MAP_UNAVAILABLE;
    InspectionMap latest;
    std::vector<InspectionMap> history;
    std::string rejected_input;
    std_msgs::msg::Header rejected_header;
    {
      std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      latest = {occupancy_map_, occupancy_map_header_, last_octomap_received_.seconds()};
      rejected_input = last_octomap_rejection_;
      rejected_header = last_rejected_octomap_header_;
      for (const auto & map : inspection_history_) {
        if (map.header.frame_id == latest.header.frame_id && inspectionMapFresh(map)) {
          history.push_back(map);
        }
      }
    }
    response->map_header = latest.tree ? latest.header : rejected_header;
    const auto map_issue = inspectionMapIssue(latest);
    if (!map_issue.empty()) {
      response->message = "Inspection requires a fresh, source-stamped OctoMap: " + map_issue;
      if (!rejected_input.empty()) {
        response->message += "; last rejected input: " + rejected_input;
      }
      return;
    }
    try {
      if (!std::isfinite(request->minimum_radius) || request->minimum_radius < 0.0 ||
        !std::isfinite(request->minimum_agl) || request->minimum_agl < 0.0 ||
        !std::isfinite(request->initial_climb_tracking_tolerance) ||
        request->initial_climb_tracking_tolerance < 0.0 ||
        request->initial_climb_tracking_tolerance > 0.1 ||
        (request->allow_terrain_escape &&
        (!request->terrain_envelope || request->minimum_agl <= 0.0)) ||
        request->corridor.poses.size() > 512)
      {
        throw std::invalid_argument("Inspection clearances or corridor size are invalid");
      }
      if (!std::isfinite(request->smoke_wall_width) || request->smoke_wall_width < 0.0 ||
        request->smoke_wall_width > 3.0 ||
        (request->smoke_wall_width > 0.0 && request->direction.header.frame_id.empty()))
      {
        throw std::invalid_argument("Smoke wall width must be between zero and 3 metres");
      }
      const auto & frame = latest.header.frame_id;
      if (!request->direction.header.frame_id.empty()) {
        if (!std::isfinite(request->max_range) || request->max_range <= 0.0 ||
          request->max_range > inspection_max_range_)
        {
          throw std::invalid_argument("Inspection ray range is outside configured limits");
        }
        const auto origin = inspectionPoint(request->origin, frame);
        geometry_msgs::msg::Vector3Stamped direction;
        const auto & supplied = request->direction.vector;
        if (!std::isfinite(supplied.x) || !std::isfinite(supplied.y) ||
          !std::isfinite(supplied.z) ||
          std::hypot(std::hypot(supplied.x, supplied.y), supplied.z) < 1e-9)
        {
          throw std::invalid_argument("Inspection direction must be finite and nonzero");
        }
        tf_buffer_->transform(request->direction, direction, frame,
          tf2::durationFromSec(transform_timeout_));
        if (!std::isfinite(direction.vector.x) || !std::isfinite(direction.vector.y) ||
          !std::isfinite(direction.vector.z))
        {
          throw std::invalid_argument("Transformed inspection direction is invalid");
        }
        const octomap::point3d ray_origin(origin.x, origin.y, origin.z);
        octomap::point3d ray_direction(direction.vector.x, direction.vector.y, direction.vector.z);
        ray_direction.normalize();
        response->ray_origin.header = request->origin.header;
        response->ray_origin.header.frame_id = frame;
        response->ray_origin.point = origin;
        response->ray_direction.header = request->direction.header;
        response->ray_direction.header.frame_id = frame;
        response->ray_direction.vector.x = ray_direction.x();
        response->ray_direction.vector.y = ray_direction.y();
        response->ray_direction.vector.z = ray_direction.z();
        if (request->smoke_wall_width > 0.0) {
          // Sample a physical 3-D aperture, not an angular cone or an inferred
          // emitter position. Missing/unknown voxels are never free evidence.
          geometry_msgs::msg::Vector3Stamped camera_y, camera_z, side, up;
          camera_y.header = camera_z.header = request->direction.header;
          camera_y.vector.y = camera_z.vector.z = 1.0;
          tf_buffer_->transform(camera_y, side, frame, tf2::durationFromSec(transform_timeout_));
          tf_buffer_->transform(camera_z, up, frame, tf2::durationFromSec(transform_timeout_));
          const octomap::point3d reference_up(up.vector.x, up.vector.y, up.vector.z);
          octomap::point3d lateral = reference_up.cross(ray_direction);
          if (lateral.norm() < 1e-6) {
            const octomap::point3d reference_side(side.vector.x, side.vector.y, side.vector.z);
            lateral = reference_side - ray_direction * reference_side.dot(ray_direction);
          }
          if (lateral.norm() < 1e-6) {
            throw std::invalid_argument("Smoke wall aperture has no transverse axis");
          }
          lateral.normalize();
          octomap::point3d vertical = ray_direction.cross(lateral);
          vertical.normalize();
          std::vector<double> ranges;
          std::set<int> rows, columns;
          double voxel_uncertainty = 0.0;
          for (int row = -2; row <= 2; ++row) {
            for (int column = -2; column <= 2; ++column) {
              const auto sample = ray_origin + lateral *
                (column * request->smoke_wall_width / 4.0) +
                vertical * (row * request->smoke_wall_width / 4.0);
              double nearest = request->max_range + 1.0;
              for (const auto & map : history) {
                octomap::point3d wall_hit;
                if (!map.tree->castRay(sample, ray_direction, wall_hit, true, request->max_range)) {
                  continue;
                }
                uint32_t occupied = 0, free = 0;
                occupancyEvidence(history, wall_hit, occupied, free);
                const double depth = (wall_hit - sample).dot(ray_direction);
                if (occupied > 0 && free > 0 && depth > 0.0 && depth < nearest) {
                  nearest = depth;
                  voxel_uncertainty = std::max(voxel_uncertainty,
                    std::sqrt(3.0) * leafSizeAt(*map.tree, wall_hit) * 0.5);
                }
              }
              if (nearest <= request->max_range) {
                ranges.push_back(nearest);
                rows.insert(row);
                columns.insert(column);
              }
            }
          }
          response->smoke_wall_samples = ranges.size();
          if (ranges.size() >= 3 && rows.size() >= 2 && columns.size() >= 2) {
            double sum = 0.0;
            for (const double range : ranges) {sum += range;}
            const double mean = sum / ranges.size();
            double variance = 0.0;
            for (const double range : ranges) {variance += (range - mean) * (range - mean);}
            response->smoke_wall_range = mean;
            response->smoke_wall_nearest_range = *std::min_element(ranges.begin(), ranges.end());
            response->smoke_wall_uncertainty = voxel_uncertainty +
              std::sqrt(variance / ranges.size());
            response->smoke_wall_valid = true;
          }
        }
        octomap::point3d hit;
        if (latest.tree->castRay(ray_origin, ray_direction, hit, false, request->max_range)) {
          response->surface_valid = true;
          response->surface.header = latest.header;
          response->surface.point.x = hit.x();
          response->surface.point.y = hit.y();
          response->surface.point.z = hit.z();
          response->surface_range = (hit - ray_origin).norm();
          response->surface_uncertainty = std::sqrt(3.0) * leafSizeAt(*latest.tree, hit) * 0.5;
          occupancyEvidence(history, hit, response->occupied_samples, response->free_samples);
          response->plume_candidate = response->occupied_samples > 0 && response->free_samples > 0;
        }
      }
      if (!request->ground_query.header.frame_id.empty()) {
        const auto query = inspectionPoint(request->ground_query, frame);
        const auto support = request->terrain_envelope ?
          inspectionTerrainEnvelope(latest, history, query, true) :
          inspectionSupport(latest, history, query);
        response->ground_valid = support.valid;
        if (support.valid) {
          response->ground_surface.header = latest.header;
          response->ground_surface.point = support.point;
          response->ground_uncertainty = support.uncertainty;
        }
      }
      if (!request->corridor.poses.empty()) {
        std::vector<geometry_msgs::msg::Point> vertices;
        for (const auto & stamped : request->corridor.poses) {
          if (!validPose(stamped.pose)) {
            throw std::invalid_argument("Inspection corridor contains an invalid pose");
          }
          geometry_msgs::msg::PointStamped point;
          point.header =
            stamped.header.frame_id.empty() ? request->corridor.header : stamped.header;
          point.point = stamped.pose.position;
          vertices.push_back(inspectionPoint(point, frame));
        }
        geometry_msgs::msg::Point center;
        if (request->minimum_radius > 0.0) {
          center = inspectionPoint(request->keepout_center, frame);
        }
        const auto axis = inspectionAxis(request->keepout_axis, frame);
        geometry_msgs::msg::Point escape_origin;
        const bool anchored = !request->escape_origin.header.frame_id.empty();
        if (anchored) {
          escape_origin = inspectionPoint(request->escape_origin, frame);
        }
        if (request->minimum_agl > 0.0) {
          std::string body_reason;
          const auto & actual = vertices.front();
          if (inspectionBodyFree(latest, history, actual, body_reason)) {
            response->current_clearance_reached = inspectionGroundClearance(
              latest, history, actual, request->terrain_envelope) + 1e-6 >= request->minimum_agl;
          }
        }
        response->corridor_safe = inspectionCorridor(latest, history, vertices, center,
            request->minimum_radius, request->minimum_agl, response->message, axis,
            request->allow_initial_climb,
            request->initial_climb_tracking_tolerance, request->terrain_envelope,
            request->allow_terrain_escape, anchored ? &escape_origin : nullptr,
            request->terrain_escape_complete || response->current_clearance_reached);
      } else {
        response->message = "Inspection geometry queried; support may be canopy, not bare ground";
      }
      if (!inspectionMapFresh(latest)) {
        response->surface_valid = response->ground_valid = response->corridor_safe = false;
        response->current_clearance_reached = false;
        response->message = "Inspection OctoMap became stale while querying geometry: " +
          inspectionMapIssue(latest);
        return;
      }
      response->success = true;
      response->error_code = Response::OK;
    } catch (const tf2::TransformException & error) {
      response->surface_valid = response->ground_valid = response->corridor_safe = false;
      response->current_clearance_reached = false;
      response->error_code = Response::TRANSFORM_ERROR;
      response->message = std::string("Inspection transform failed: ") + error.what();
    } catch (const std::invalid_argument & error) {
      response->surface_valid = response->ground_valid = response->corridor_safe = false;
      response->current_clearance_reached = false;
      response->error_code = Response::INVALID_REQUEST;
      response->message = error.what();
    }
  }

  bool failPlanning(const std::string & reason)
  {
    last_planning_error_ = reason;
    RCLCPP_WARN(this->get_logger(), "Cannot plan: %s", reason.c_str());
    return false;
  }

  void planPathCallback(
    const kv_executive::srv::PlanPath::Request::SharedPtr request,
    kv_executive::srv::PlanPath::Response::SharedPtr response)
  {
    (void)request;
    response->success = planPath(response->path);
    if (response->success) {
      response->message = "Path planned successfully";
    } else {
      response->message = last_planning_error_;
    }
  }

  static bool validPose(const geometry_msgs::msg::Pose & pose)
  {
    const auto & p = pose.position;
    const auto & q = pose.orientation;
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
           std::isfinite(q.w) && std::isfinite(norm_squared) && norm_squared > 1e-12;
  }

  static void normalizeOrientation(geometry_msgs::msg::Quaternion & quaternion)
  {
    const double norm = std::sqrt(
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w);
    quaternion.x /= norm;
    quaternion.y /= norm;
    quaternion.z /= norm;
    quaternion.w /= norm;
  }

  bool planToPose(
    const geometry_msgs::msg::PoseStamped & target,
    nav_msgs::msg::Path & path_message)
  {
    last_planning_error_.clear();
    if (target.header.frame_id.empty() || !validPose(target.pose)) {
      return failPlanning(
        "Target requires a frame_id, finite position, and nonzero finite quaternion");
    }
    if (!updatePlanningPoses()) {
      return false;
    }
    std::string planning_frame;
    {
      std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      planning_frame = occupancy_map_frame_;
    }
    geometry_msgs::msg::PoseStamped transformed_target;
    try {
      // PoseStamped's stamp deliberately selects the transform at request time;
      // a zero stamp retains the standard TF convention of using latest data.
      auto normalized_target = target;
      normalizeOrientation(normalized_target.pose.orientation);
      tf_buffer_->transform(
                normalized_target, transformed_target, planning_frame,
                tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException & exception) {
      return failPlanning(
        "Cannot transform target into OctoMap frame '" + planning_frame +
        "': " + exception.what());
    }
    if (!validPose(transformed_target.pose)) {
      return failPlanning("Transformed target pose is invalid");
    }
    const auto & target_position = transformed_target.pose.position;
    const double distance = std::hypot(
      std::hypot(target_position.x - current_pose.position.x,
      target_position.y - current_pose.position.y),
      target_position.z - current_pose.position.z);
    if (!std::isfinite(distance) || distance > maximum_target_distance_) {
      return failPlanning("Target exceeds maximum_target_distance");
    }
    normalizeOrientation(transformed_target.pose.orientation);
    return createPath(
      target_position, path_message, &transformed_target.pose.orientation);
  }

  void planToPoseCallback(
    const kv_executive::srv::PlanToPose::Request::SharedPtr request,
    kv_executive::srv::PlanToPose::Response::SharedPtr response)
  {
    response->success = planToPose(request->target, response->path);
    response->message = response->success ? "Path to requested pose planned successfully" :
      last_planning_error_;
  }

  static geometry_msgs::msg::Point inspectionStatePoint(const ob::State * state)
  {
    const auto * compound = state->as<ob::CompoundStateSpace::StateType>();
    const auto * position = compound->as<ob::RealVectorStateSpace::StateType>(0);
    geometry_msgs::msg::Point point;
    point.x = position->values[0];
    point.y = position->values[1];
    point.z = position->values[2];
    return point;
  }

  void planInspectionPathCallback(
    const kv_executive::srv::PlanInspectionPath::Request::SharedPtr request,
    kv_executive::srv::PlanInspectionPath::Response::SharedPtr response)
  {
    using Response = kv_executive::srv::PlanInspectionPath::Response;
    response->success = false;
    response->error_code = Response::OK;
    last_planning_error_.clear();
    try {
      if (!validPose(request->target.pose) || request->target.header.frame_id.empty() ||
        !std::isfinite(request->minimum_radius) || request->minimum_radius < 0.0 ||
        !std::isfinite(request->minimum_agl) || request->minimum_agl < 0.0 ||
        (request->allow_terrain_escape &&
        (!request->terrain_envelope || request->minimum_agl <= 0.0)))
      {
        throw std::invalid_argument("Inspection target or clearances are invalid");
      }
      if (!updatePlanningPoses()) {
        response->message = last_planning_error_;
        response->error_code = last_planning_error_.find("Cannot transform") == 0 ?
          Response::TRANSFORM_ERROR : Response::MAP_UNAVAILABLE;
        return;
      }
      InspectionMap latest;
      std::vector<InspectionMap> history;
      {
        std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
        latest = {occupancy_map_, occupancy_map_header_, last_octomap_received_.seconds()};
        for (const auto & map : inspection_history_) {
          if (map.header.frame_id == latest.header.frame_id && inspectionMapFresh(map)) {
            history.push_back(map);
          }
        }
      }
      if (!inspectionMapFresh(latest)) {
        response->error_code = Response::MAP_UNAVAILABLE;
        response->message = "Inspection planning requires a fresh, source-stamped OctoMap: " +
          inspectionMapIssue(latest);
        return;
      }
      const auto & frame = latest.header.frame_id;
      auto supplied_target = request->target;
      normalizeOrientation(supplied_target.pose.orientation);
      geometry_msgs::msg::PoseStamped target;
      tf_buffer_->transform(supplied_target, target, frame,
        tf2::durationFromSec(transform_timeout_));
      if (!validPose(target.pose)) {
        throw std::invalid_argument("Transformed inspection target is invalid");
      }
      const auto & original = current_pose;
      const auto & end = target.pose.position;
      std::string start_reason;
      if (!inspectionBodyFree(latest, history, original.position, start_reason)) {
        response->error_code = Response::START_INVALID;
        response->message = "OMPL start state is invalid: " + start_reason;
        return;
      }
      if (std::hypot(std::hypot(end.x - original.position.x, end.y - original.position.y),
        end.z - original.position.z) > maximum_target_distance_)
      {
        response->message = "Inspection target exceeds maximum_target_distance";
        return;
      }
      const auto axis = inspectionAxis(request->keepout_axis, frame);
      geometry_msgs::msg::Point center;
      if (request->minimum_radius > 0.0) {
        center = inspectionPoint(request->keepout_center, frame);
      }
      geometry_msgs::msg::Pose initial = original;
      bool climbed = false;
      std::vector<geometry_msgs::msg::Point> initial_prefix{original.position};
      if ((request->allow_initial_climb || request->allow_terrain_escape) &&
        request->minimum_agl > 0.0)
      {
        const double agl = inspectionGroundClearance(
          latest, history, original.position, request->terrain_envelope);
        if (std::isnan(agl)) {
          response->error_code = Response::START_INVALID;
          response->message =
            "Inspection initial climb requires an observed stable support surface";
          return;
        }
        if (agl + 1e-6 < request->minimum_agl) {
          if (axis.z <= 1e-6) {
            response->message = "Inspection initial climb axis does not point upward";
            return;
          }
          const double floor = home_pose.z + minimum_altitude_ + bound_margin_;
          const double ceiling = home_pose.z + maximum_altitude_ - bound_margin_;
          if (original.position.z < floor) {
            response->error_code = Response::START_INVALID;
            response->message = "Inspection initial climb violates home-relative altitude limits";
            return;
          }
          std::string reason;
          const auto try_prefix = [&](const geometry_msgs::msg::Point & side) {
              const auto side_support = request->terrain_envelope ?
                inspectionTerrainEnvelope(latest, history, side) :
                inspectionSupport(latest, history, side);
              if (!side_support.valid) {
                return false;
              }
              auto raised = side;
              const double side_agl = side.z - side_support.point.z - side_support.uncertainty;
              const double distance = std::max(0.0,
                  (request->minimum_agl - side_agl + 1e-4) / axis.z);
              raised.x += distance * axis.x;
              raised.y += distance * axis.y;
              raised.z += distance * axis.z;
              if (raised.z > ceiling) {
                reason = "Inspection initial climb violates home-relative altitude limits";
                return false;
              }
              std::vector<geometry_msgs::msg::Point> prefix{original.position};
              if (std::hypot(std::hypot(side.x - original.position.x,
                side.y - original.position.y), side.z - original.position.z) > 1e-9)
              {
                prefix.push_back(side);
              }
              prefix.push_back(raised);
              if (!inspectionCorridor(latest, history, prefix, center,
                request->minimum_radius, request->minimum_agl, reason, axis, true, 0.0,
                request->terrain_envelope, request->allow_terrain_escape))
              {
                return false;
              }
              initial_prefix = std::move(prefix);
              initial.position = raised;
              return true;
            };
          bool escaped = try_prefix(original.position);
          const std::string direct_reason = reason;
          // A separate opt-in permits a short lateral connector before the
          // climb. All samples stay observed free and at nondecreasing measured
          // clearance. This is never a waiver for an unknown starting body.
          for (const double radius : {0.5, 1.0, 2.0}) {
            if (escaped || !request->allow_terrain_escape) {
              break;
            }
            for (unsigned direction = 0; direction < 8 && !escaped; ++direction) {
              auto side = original.position;
              const double angle = direction * 2.0 * M_PI / 8.0;
              side.x += radius * std::cos(angle);
              side.y += radius * std::sin(angle);
              escaped = try_prefix(side);
            }
          }
          if (!escaped && request->allow_terrain_escape && inspectionClearanceEscape(
            latest, history, original.position, center, request->minimum_radius,
            request->minimum_agl, axis, ceiling, initial_prefix, reason))
          {
            initial.position = initial_prefix.back();
            escaped = true;
          }
          if (!escaped) {
            response->error_code = Response::NO_CLEARANCE_ESCAPE;
            response->message = "No observed-free initial clearance escape; direct climb: " +
              direct_reason + "; recovery: " + reason;
            return;
          }
          climbed = true;
        }
      }
      const auto last_reason = std::make_shared<std::string>();
      const auto validity = [this, latest, history, center, axis, request, last_reason](
        const ob::State * state) {
          if (!space_->satisfiesBounds(state)) {
            *last_reason = "Inspection state violates planning bounds";
            return false;
          }
          return inspectionCorridor(latest, history, {inspectionStatePoint(state)}, center,
            request->minimum_radius, request->minimum_agl, *last_reason, axis, false, 0.0,
            request->terrain_envelope);
        };
      const auto motion = [this, latest, history, center, axis, request, last_reason](
        const ob::State * start, const ob::State * finish) {
          return inspectionCorridor(latest, history,
                   {inspectionStatePoint(start), inspectionStatePoint(finish)}, center,
            request->minimum_radius, request->minimum_agl, *last_reason, axis, false, 0.0,
            request->terrain_envelope);
        };
      if (!createPath(end, response->path, &target.pose.orientation, validity, motion,
        climbed ? &initial : nullptr, false))
      {
        response->message = last_planning_error_;
        if (last_planning_error_.find("OMPL start state is invalid") == 0) {
          response->error_code = Response::START_INVALID;
        } else if (last_planning_error_.find("OMPL goal state is invalid") == 0) {
          response->error_code = Response::GOAL_INVALID;
        }
        if (!last_reason->empty()) {
          response->message += ": " + *last_reason;
        }
        return;
      }
      if (climbed) {
        std::vector<geometry_msgs::msg::PoseStamped> prefix;
        for (std::size_t index = 0; index + 1 < initial_prefix.size(); ++index) {
          geometry_msgs::msg::PoseStamped vertex;
          vertex.header = response->path.header;
          vertex.pose = original;
          vertex.pose.position = initial_prefix[index];
          prefix.push_back(vertex);
        }
        response->path.poses.insert(response->path.poses.begin(), prefix.begin(), prefix.end());
      }
      std::vector<geometry_msgs::msg::Point> vertices;
      for (const auto & vertex : response->path.poses) {
        vertices.push_back(vertex.pose.position);
      }
      if (!inspectionCorridor(latest, history, vertices, center, request->minimum_radius,
        request->minimum_agl, response->message, axis, request->allow_initial_climb, 0.0,
        request->terrain_envelope, request->allow_terrain_escape) ||
        !inspectionMapFresh(latest))
      {
        response->path.poses.clear();
        if (!inspectionMapFresh(latest)) {
          response->error_code = Response::MAP_UNAVAILABLE;
          response->message = "Inspection OctoMap became stale while planning: " +
            inspectionMapIssue(latest);
        }
        return;
      }
      response->success = true;
      response->message = inspection_require_observed_free_space_ ?
        "Strict inspection path planned through observed free space" :
        "Inspection path planned clear of current and recent occupied voxels";
      path_publisher_->publish(response->path);
    } catch (const tf2::TransformException & error) {
      response->error_code = Response::TRANSFORM_ERROR;
      response->message = std::string("Inspection planning transform failed: ") + error.what();
    } catch (const std::invalid_argument & error) {
      response->error_code = Response::INVALID_REQUEST;
      response->message = error.what();
    }
  }

  void fireSourceCallback(const kv_executive::msg::FireSourceReport::SharedPtr report)
  {
    if (report->planning_pose.header.frame_id.empty() ||
      !validPose(report->planning_pose.pose) || !std::isfinite(report->exclusion_radius) ||
      report->exclusion_radius < 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "Ignoring invalid fire-source region report");
      return;
    }
    // Reports describe the safe observation viewpoint, not a measured 3-D fire
    // location. They exclude future exploration goals, never transit/escape paths.
    const auto existing = std::find_if(
      reported_sites_.begin(), reported_sites_.end(),
      [&report](const auto & site) {return site.site_id == report->site_id;});
    if (existing != reported_sites_.end()) {
      reported_sites_.erase(existing);
    }
    reported_sites_.push_back(*report);
    if (reported_sites_.size() > maximum_reported_sites_) {
      reported_sites_.erase(reported_sites_.begin());
    }
    RCLCPP_INFO(this->get_logger(), "Recorded fire-source exploration exclusion region");
  }

  void updateReportedSiteRegions()
  {
    reported_site_regions_.clear();
    std::string planning_frame;
    {
      std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      planning_frame = occupancy_map_frame_;
    }
    for (const auto & site : reported_sites_) {
      geometry_msgs::msg::PoseStamped site_pose;
      try {
        tf_buffer_->transform(
                  site.planning_pose, site_pose, planning_frame,
                  tf2::durationFromSec(0.0));
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Ignoring fire-source region without a transform into '%s': %s",
          planning_frame.c_str(), exception.what());
        continue;
      }
      const double radius = site.exclusion_radius > 0.0 ? site.exclusion_radius :
        reported_site_exclusion_radius_;
      if (!validPose(site_pose.pose)) {
        RCLCPP_WARN(this->get_logger(), "Ignoring invalid transformed fire-source region");
        continue;
      }
      reported_site_regions_.emplace_back(site_pose.pose.position, radius);
    }
  }

  bool outsideReportedSites(const geometry_msgs::msg::Point & candidate) const
  {
    for (const auto & region : reported_site_regions_) {
      if (std::hypot(
          candidate.x - region.first.x,
          candidate.y - region.first.y) < region.second)
      {
        return false;
      }
    }
    return true;
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odometry_ = *msg;
    odometry_received_ = true;

    if (home_from_first_odometry_ && !home_pose_source_received_) {
      home_pose_source_.header = msg->header;
      home_pose_source_.pose = msg->pose.pose;
      home_pose_source_received_ = true;
      RCLCPP_INFO(
                this->get_logger(),
                "Latched home from first odometry sample in frame '%s'",
                msg->header.frame_id.c_str());
    }

    RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "External odometry: frame='%s', child='%s', position=(%.2f, %.2f, %.2f)",
            msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  void homePositionCallback(const mavros_msgs::msg::HomePosition::SharedPtr msg)
  {
    home_pose_source_.header = msg->header;
    home_pose_source_.pose.position = msg->position;
    home_pose_source_.pose.orientation = msg->orientation;
    home_pose_source_received_ = true;

    RCLCPP_INFO(
            this->get_logger(), "MAVROS home received in frame '%s': (%.2f, %.2f, %.2f)",
            msg->header.frame_id.c_str(), msg->position.x, msg->position.y, msg->position.z);
  }

  bool updatePlanningPoses()
  {
    std::string missing;
    const auto add_missing = [&missing](const std::string & input) {
        if (!missing.empty()) {
          missing += ", ";
        }
        missing += input;
      };
    if (!odometry_received_) {
      add_missing("odometry (" + odometry_topic_ + ")");
    }
    if (!home_pose_source_received_) {
      add_missing("home (" +
        (home_from_first_odometry_ ? odometry_topic_ : home_position_topic_) + ")");
    }
    if (!occupancy_map_received_) {
      add_missing("OctoMap (" + octomap_topic_ + ")");
    }
    if (!missing.empty()) {
      return failPlanning("Missing inputs: " + missing);
    }

    std::string planning_frame;
    {
      std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      planning_frame = occupancy_map_frame_;
    }
    if (planning_frame.empty()) {
      return failPlanning("OctoMap frame is empty");
    }

    geometry_msgs::msg::PoseStamped odometry_pose;
    odometry_pose.header = latest_odometry_.header;
    odometry_pose.pose = latest_odometry_.pose.pose;
    if (odometry_pose.header.frame_id.empty()) {
      return failPlanning("Odometry frame_id is empty on " + odometry_topic_);
    }
    if (home_pose_source_.header.frame_id.empty()) {
      return failPlanning("Home frame_id is empty");
    }
    if (!validPose(odometry_pose.pose) || !validPose(home_pose_source_.pose)) {
      return failPlanning(
        "Odometry/home pose requires finite position and nonzero finite quaternion");
    }

    try {
      geometry_msgs::msg::PoseStamped current_pose_in_planning_frame;
      geometry_msgs::msg::PoseStamped home_pose_in_planning_frame;
      auto home_pose_for_transform = home_pose_source_;
      normalizeOrientation(odometry_pose.pose.orientation);
      normalizeOrientation(home_pose_for_transform.pose.orientation);
      if (home_from_first_odometry_) {
        // A map-to-odometry correction may change over time. Apply its current
        // value to the latched home coordinate instead of requesting a TF that
        // may already be older than the buffer cache.
        home_pose_for_transform.header.stamp = odometry_pose.header.stamp;
      }
      tf_buffer_->transform(
                odometry_pose, current_pose_in_planning_frame, planning_frame,
                tf2::durationFromSec(transform_timeout_));
      tf_buffer_->transform(
                home_pose_for_transform, home_pose_in_planning_frame, planning_frame,
                tf2::durationFromSec(transform_timeout_));

      current_pose = current_pose_in_planning_frame.pose;
      home_pose = home_pose_in_planning_frame.pose.position;
      if (!validPose(current_pose) || !validPose(home_pose_in_planning_frame.pose)) {
        return failPlanning("Transformed odometry/home pose is invalid");
      }
      normalizeOrientation(current_pose.orientation);
      current_pose_received_ = true;
      home_pose_received_ = true;
    } catch (const tf2::TransformException & exception) {
      current_pose_received_ = false;
      home_pose_received_ = false;
      return failPlanning(
        "Cannot transform odometry/home into OctoMap frame '" + planning_frame +
        "': " + exception.what());
    }

    RCLCPP_INFO(
            this->get_logger(),
            "Planning pose in '%s': current=(%.2f, %.2f, %.2f), home_z=%.2f",
            planning_frame.c_str(), current_pose.position.x, current_pose.position.y,
            current_pose.position.z, home_pose.z);
    return true;
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    const auto received = octomap_clock_.now();
    const double source = stampSeconds(msg->header.stamp);
    const double ros_before = this->get_clock()->now().seconds();
    double decode_elapsed = 0.0;
    const auto reject = [&](const std::string & reason, std::size_t nodes = 0) {
        const double elapsed = (octomap_clock_.now() - received).seconds();
        const auto detail = mapAgeDetail({nullptr, msg->header, received.seconds()});
        {
          std::unique_lock<std::shared_mutex> lock(occupancy_map_mutex_);
          last_octomap_rejection_ = reason + detail;
          last_rejected_octomap_header_ = msg->header;
        }
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), octomap_clock_, 1000,
          "Rejected OctoMap: %s; bytes=%zu, nodes=%zu, decode_elapsed=%.6f s, "
          "callback_elapsed=%.6f s, source_age_before=%.6f s, source_age_after=%.6f s%s",
          reason.c_str(), msg->data.size(), nodes, decode_elapsed, elapsed, ros_before - source,
          this->get_clock()->now().seconds() - source,
          detail.c_str());
      };
    if (msg->header.frame_id.empty()) {
      reject("missing map frame_id");
      return;
    }
    // Legacy exploration maps can be unstamped. Inspection never accepts them.
    // Discard stamped backlog before allocating another complete decoded tree.
    if (source > 0.0 && ros_before > 0.0) {
      const auto issue = inspectionSourceIssue(msg->header);
      if (!issue.empty()) {
        reject(issue);
        return;
      }
    }
    bool clock_restarted = false;
    bool out_of_order = false;
    bool missing_replacement_stamp = false;
    {
      std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      const auto & previous = occupancy_map_header_.stamp;
      clock_restarted = ros_before > 0.0 && stampSeconds(previous) >
        ros_before + inspection_future_tolerance_;
      const bool newer = msg->header.stamp.sec > previous.sec ||
        (msg->header.stamp.sec == previous.sec && msg->header.stamp.nanosec > previous.nanosec);
      out_of_order = source > 0.0 && occupancy_map_ &&
        msg->header.frame_id == occupancy_map_header_.frame_id && !clock_restarted && !newer;
      missing_replacement_stamp = source <= 0.0 && stampSeconds(previous) > 0.0;
    }
    if (out_of_order || missing_replacement_stamp) {
      reject(missing_replacement_stamp ?
        "zero source stamp cannot replace a stamped map" :
        "duplicate or out-of-order source stamp; keeping the newer accepted map");
      return;
    }
    if (msg->data.empty()) {
      reject("empty serialized map data");
      return;
    }
    const auto decode_start = octomap_clock_.now();
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
    decode_elapsed = (octomap_clock_.now() - decode_start).seconds();
    if (!abstract_tree) {
      reject("failed to deserialize map data");
      return;
    }

    auto *octree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (!octree) {
      reject("unsupported tree type: " + msg->id);
      return;
    }
    if (octree->size() == 0) {
      reject("decoded tree is empty");
      return;
    }
    const auto nodes = octree->size();
    if (source > 0.0 && this->get_clock()->now().seconds() > 0.0) {
      const auto issue = inspectionSourceIssue(msg->header);
      if (!issue.empty()) {
        reject("map became unusable while decoding: " + issue, nodes);
        return;
      }
    }
    if (clock_restarted && source <= 0.0) {
      reject("ROS clock restarted but replacement source stamp is unavailable", nodes);
      return;
    }

    std::shared_ptr<const octomap::OcTree> new_map(
      static_cast<octomap::OcTree *>(abstract_tree.release()));
    {
      std::unique_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      occupancy_map_ = std::move(new_map);
      occupancy_map_frame_ = msg->header.frame_id;
      occupancy_map_received_ = true;
      last_octomap_received_ = received;
      occupancy_map_header_ = msg->header;
      last_octomap_rejection_.clear();
      last_rejected_octomap_header_ = std_msgs::msg::Header();
      if (clock_restarted || (!inspection_history_.empty() &&
        (inspection_history_.back().header.frame_id != msg->header.frame_id ||
        inspection_history_.back().tree->getResolution() != occupancy_map_->getResolution())))
      {
        inspection_history_.clear();
      }
      if (stampSeconds(msg->header.stamp) > 0.0 &&
        (inspection_history_.empty() || stampSeconds(msg->header.stamp) >
        stampSeconds(inspection_history_.back().header.stamp)))
      {
        inspection_history_.push_back(
          {occupancy_map_, msg->header, last_octomap_received_.seconds()});
      }
      // Reclaim source-expired snapshots, not temporarily unusable ones. A
      // paused /clock blocks inspection but must not erase occupied evidence
      // that will still be source-fresh when simulation resumes.
      while (!inspection_history_.empty() &&
        (inspection_history_.size() > 8 ||
        !inspectionSourceIssue(inspection_history_.front().header).empty()))
      {
        inspection_history_.pop_front();
      }
    }

    RCLCPP_INFO_THROTTLE(
            this->get_logger(), octomap_clock_, 1000,
            "Received binary OctoMap: frame=%s, resolution=%.3f m, bytes=%zu, nodes=%zu, "
            "decode_elapsed=%.6f s, callback_elapsed=%.6f s, source_age_before=%.6f s, "
            "source_age_after=%.6f s, clock_restarted=%s",
            msg->header.frame_id.c_str(), msg->resolution, msg->data.size(), nodes,
            decode_elapsed, (octomap_clock_.now() - received).seconds(), ros_before - source,
            this->get_clock()->now().seconds() - source, clock_restarted ? "true" : "false");
  }

  void checkOctomapReception()
  {
    InspectionMap latest;
    {
      std::unique_lock<std::shared_mutex> lock(occupancy_map_mutex_);
      latest = {occupancy_map_, occupancy_map_header_, last_octomap_received_.seconds()};
      while (!inspection_history_.empty() &&
        !inspectionSourceIssue(inspection_history_.front().header).empty())
      {
        inspection_history_.pop_front();
      }
    }
    const double age = octomap_clock_.now().seconds() - latest.received;
    const auto issue = inspectionMapIssue(latest);
    if (!issue.empty() && (latest.tree || age >= inspection_map_timeout_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), octomap_clock_, 5000,
        "OctoMap unavailable for inspection on '%s': %s",
        octomap_topic_.c_str(), issue.c_str());
    }
  }

  void loadPlannerParameters()
  {
    horizontal_range_ = this->declare_parameter<double>("horizontal_range");
    vertical_range_ = this->declare_parameter<double>("vertical_range");
    minimum_altitude_ = this->declare_parameter<double>("minimum_altitude");
    maximum_altitude_ = this->declare_parameter<double>("maximum_altitude");
    bound_margin_ = this->declare_parameter<double>("bound_margin");
    yaw_weight_ = this->declare_parameter<double>("yaw_weight");
    drone_radius_ = this->declare_parameter<double>("drone_radius");
    obstacle_clearance_ = this->declare_parameter<double>("obstacle_clearance");
    validity_resolution_fraction_ =
      this->declare_parameter<double>("validity_resolution_fraction");
    planning_timeout_ = this->declare_parameter<double>("planning_timeout");
    simplification_time_ = this->declare_parameter<double>("simplification_time");
    odometry_topic_ = this->declare_parameter<std::string>(
            "odometry_topic");
    octomap_topic_ = this->declare_parameter<std::string>("octomap_topic", "/octomap_binary");
    home_position_topic_ = this->declare_parameter<std::string>(
            "home_position_topic", "/mavros/home_position/home");
    mavros_parameter_service_ = this->declare_parameter<std::string>(
            "mavros_parameter_service", "/mavros/param/get_parameters");
    home_from_first_odometry_ = this->declare_parameter<bool>(
            "home_from_first_odometry", true);
    transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.2);
    inspection_map_timeout_ = this->declare_parameter<double>("inspection_map_timeout", 5.0);
    inspection_require_observed_free_space_ = this->declare_parameter<bool>(
      "inspection_require_observed_free_space", false);
    inspection_future_tolerance_ =
      this->declare_parameter<double>("inspection_future_tolerance", 0.25);
    inspection_max_range_ = this->declare_parameter<double>("inspection_max_range", 50.0);
    inspection_ground_patch_radius_ =
      this->declare_parameter<double>("inspection_ground_patch_radius", 0.5);
    if (!std::isfinite(inspection_map_timeout_) || inspection_map_timeout_ <= 0.0 ||
      !std::isfinite(inspection_future_tolerance_) || inspection_future_tolerance_ < 0.0 ||
      !std::isfinite(inspection_max_range_) || inspection_max_range_ <= 0.0 ||
      !std::isfinite(inspection_ground_patch_radius_) || inspection_ground_patch_radius_ <= 0.0)
    {
      throw std::invalid_argument("Inspection query limits must be finite and positive");
    }
    maximum_target_distance_ = this->declare_parameter<double>("maximum_target_distance", 50.0);
    reported_site_exclusion_radius_ =
      this->declare_parameter<double>("reported_site_exclusion_radius", 15.0);
    const auto maximum_reported_sites =
      this->declare_parameter<int>("maximum_reported_sites", 100);
    if (!std::isfinite(maximum_target_distance_) || maximum_target_distance_ <= 0.0 ||
      !std::isfinite(reported_site_exclusion_radius_) || reported_site_exclusion_radius_ <= 0.0 ||
      maximum_reported_sites < 1)
    {
      throw std::invalid_argument(
        "maximum_target_distance, reported_site_exclusion_radius, and maximum_reported_sites "
        "must be positive and finite");
    }
    maximum_reported_sites_ = static_cast<std::size_t>(maximum_reported_sites);

    if (horizontal_range_ <= 0.0 || vertical_range_ <= 0.0) {
      throw std::invalid_argument("horizontal_range and vertical_range must be greater than zero");
    }
    if (minimum_altitude_ >= maximum_altitude_) {
      throw std::invalid_argument("minimum_altitude must be less than maximum_altitude");
    }
    if (bound_margin_ < 0.0 || drone_radius_ < 0.0 || obstacle_clearance_ < 0.0) {
      throw std::invalid_argument(
        "bound_margin, drone_radius, and obstacle_clearance cannot be negative");
    }
    if (yaw_weight_ <= 0.0) {
      throw std::invalid_argument("yaw_weight must be greater than zero");
    }
    if (validity_resolution_fraction_ <= 0.0 || validity_resolution_fraction_ > 1.0) {
      throw std::invalid_argument("validity_resolution_fraction must be in the range (0, 1]");
    }
    if (planning_timeout_ <= 0.0) {
      throw std::invalid_argument("planning_timeout must be greater than zero");
    }
    if (simplification_time_ <= 0.0) {
      throw std::invalid_argument("simplification_time must be greater than zero");
    }
    if (odometry_topic_.empty() || home_position_topic_.empty() || octomap_topic_.empty() ||
      mavros_parameter_service_.empty())
    {
      throw std::invalid_argument("planner topic and service names cannot be empty");
    }
    if (transform_timeout_ < 0.0) {
      throw std::invalid_argument("transform_timeout cannot be negative");
    }
  }

  void createStateSpace()
  {
    position_space_ = std::make_shared<ob::RealVectorStateSpace>(3);
    yaw_space_ = std::make_shared<ob::SO2StateSpace>();

    space_ = std::make_shared<ob::CompoundStateSpace>();
    space_->addSubspace(position_space_, 1.0);
    space_->addSubspace(yaw_space_, yaw_weight_);
        // Prevent any more subspaces being added later.
    space_->lock();
  }

  bool updatePositionSpaceBounds(
    const geometry_msgs::msg::Point & goal_position, bool explicit_target = false,
    const geometry_msgs::msg::Pose * start_override = nullptr)
  {
    const double drone_x = current_pose.position.x;
    const double drone_y = current_pose.position.y;
    const double drone_z = current_pose.position.z;

    const double home_z = home_pose.z;

    // Exploration retains the original rolling volume. Explicit Verify/retreat
    // requests extend that volume to include the endpoint and detour space,
    // while the home-relative altitude limits remain hard constraints.
    const double min_x = explicit_target ? std::min(drone_x, goal_position.x) -
      horizontal_range_ : drone_x - horizontal_range_;
    const double max_x = explicit_target ? std::max(drone_x, goal_position.x) +
      horizontal_range_ : drone_x + horizontal_range_;
    const double min_y = explicit_target ? std::min(drone_y, goal_position.y) -
      horizontal_range_ : drone_y - horizontal_range_;
    const double max_y = explicit_target ? std::max(drone_y, goal_position.y) +
      horizontal_range_ : drone_y + horizontal_range_;
    const double initial_z = start_override ? start_override->position.z : drone_z;
    const double min_z = std::max(home_z + minimum_altitude_ + bound_margin_,
        (explicit_target ? std::min({drone_z, goal_position.z, initial_z}) : drone_z) -
        vertical_range_);
    const double max_z = std::min(home_z + maximum_altitude_ - bound_margin_,
        (explicit_target ? std::max({drone_z, goal_position.z, initial_z}) : drone_z) +
        vertical_range_);

    if (min_x >= max_x || min_y >= max_y || min_z >= max_z) {
      RCLCPP_ERROR(this->get_logger(),
                "Invalid planning bounds: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f]",
                min_x, max_x, min_y, max_y, min_z, max_z
      );
      return failPlanning("Invalid altitude or horizontal planning bounds; check home reference");
    }

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, min_x);
    bounds.setHigh(0, max_x);
    bounds.setLow(1, min_y);
    bounds.setHigh(1, max_y);
    bounds.setLow(2, min_z);
    bounds.setHigh(2, max_z);
    position_space_->setBounds(bounds);
    RCLCPP_INFO(this->get_logger(),
            "Rolling planning bounds: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f]",
            min_x, max_x, min_y, max_y, min_z, max_z
    );

    const bool goal_inside_bounds = goal_position.x >= bounds.low[0] &&
      goal_position.x <= bounds.high[0] &&
      goal_position.y >= bounds.low[1] && goal_position.y <= bounds.high[1] &&
      goal_position.z >= bounds.low[2] && goal_position.z <= bounds.high[2];
    if (!goal_inside_bounds) {
      return failPlanning("Goal is outside the current planning bounds");
    }
    return true;
  }

  bool isStateValid(const ob::State *state) const
  {
        // 1. State must lie inside the rolling planning bounds.
    if (!space_->satisfiesBounds(state)) {
      return false;
    }

    const auto *compound_state = state->as<ob::CompoundStateSpace::StateType>();
    const auto *position_state = compound_state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *yaw_state = compound_state->as<ob::SO2StateSpace::StateType>(1);

    const double x = position_state->values[0];
    const double y = position_state->values[1];
    const double z = position_state->values[2];
    const double yaw = yaw_state->value;

    return isDroneBodyCollisionFree(x, y, z, yaw);
        // hasRequiredObstacleClearance(drone_state) &&
        // isOutsideNoFlyZones(drone_state) && satisfiesOrientationConstraint(drone_state);
  }

  bool isDroneBodyCollisionFree(double x, double y, double z, double yaw) const
  {
    (void)yaw;
    const double checking_radius = drone_radius_ + obstacle_clearance_;

    std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
    return !intersectSphere(occupancy_map_, x, y, z, checking_radius);
  }

  bool intersectSphere(
    const std::shared_ptr<const octomap::OcTree> & occupancy_map,
    double x, double y, double z, double radius) const
  {
    if (!occupancy_map || radius < 0.0) {
      return false;
    }

    const octomap::point3d minimum(x - radius, y - radius, z - radius);
    const octomap::point3d maximum(x + radius, y + radius, z + radius);
    const double radius_squared = radius * radius;

    for (auto iterator = occupancy_map->begin_leafs_bbx(minimum, maximum),
      end = occupancy_map->end_leafs_bbx(); iterator != end; ++iterator)
    {
      if (!occupancy_map->isNodeOccupied(*iterator)) {
        continue;
      }

            // Find the squared distance from the sphere center to the closest
            // point in this leaf voxel's axis-aligned bounding box.
      const double half_size = iterator.getSize() * 0.5;
      const double dx = std::max(std::abs(x - iterator.getX()) - half_size, 0.0);
      const double dy = std::max(std::abs(y - iterator.getY()) - half_size, 0.0);
      const double dz = std::max(std::abs(z - iterator.getZ()) - half_size, 0.0);

      if (dx * dx + dy * dy + dz * dz <= radius_squared) {
        return true;
      }
    }
    return false;
  }

  void requestMaximumAltitude()
  {
    if (!param_mavros_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
        "Waiting for MAVROS parameter service..."
      );
      return;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        // It is currently limited to PX4. Ardupilot will use the default maximum_altitude_.
    request->names.push_back("GF_MAX_VER_DIST");

    param_mavros_->async_send_request(
            request,
      [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future)
      {
        const auto response = future.get();
        if (response->values.empty()) {
          RCLCPP_ERROR(this->get_logger(), "MAVROS returned no value for MAXIMUM ALTITUDE");
          return;
        }
        const auto & value = response->values.front();
        if(value.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
          RCLCPP_ERROR(this->get_logger(), "MAXIMUM ALTITUDE has unexpected ROS parameter type");
          return;
        }
        const double max_altitude = value.double_value;
        if (!std::isfinite(max_altitude) || max_altitude <= 0.0) {
          RCLCPP_WARN(this->get_logger(),
          "MAXIMUM ALTITUDE is disabled, using default MAXIMUM ALTITUDE");
        } else {
          RCLCPP_INFO(this->get_logger(), "Robot maximum vertical distance from Home: %.2f m",
          max_altitude);
          maximum_altitude_ = max_altitude;
        }
        timer_->cancel();
            }
    );
  }

  double horizontal_range_;
  double vertical_range_;
  double minimum_altitude_;
  double maximum_altitude_;
  double bound_margin_;
  double yaw_weight_;
  double drone_radius_;
  double obstacle_clearance_;
  double validity_resolution_fraction_;
  double planning_timeout_;
  double simplification_time_;
  double transform_timeout_;
  double inspection_map_timeout_;
  bool inspection_require_observed_free_space_;
  double inspection_future_tolerance_;
  double inspection_max_range_;
  double inspection_ground_patch_radius_;
  double maximum_target_distance_;
  double reported_site_exclusion_radius_;
  std::size_t maximum_reported_sites_;
  std::string odometry_topic_;
  std::string octomap_topic_;
  std::string last_planning_error_;
  std::string home_position_topic_;
  std::string mavros_parameter_service_;
  bool home_from_first_odometry_;
  bool odometry_received_{false};
  bool home_pose_source_received_{false};
  bool current_pose_received_{false};
  bool home_pose_received_{false};
  bool occupancy_map_received_{false};
  std::vector<kv_executive::msg::FireSourceReport> reported_sites_;
  std::vector<std::pair<geometry_msgs::msg::Point, double>> reported_site_regions_;

  std::shared_ptr<ob::RealVectorStateSpace> position_space_;
  std::shared_ptr<ob::SO2StateSpace> yaw_space_;
  std::shared_ptr<ob::CompoundStateSpace> space_;
  std::shared_ptr<og::SimpleSetup> ss_;
  std::mt19937 random_generator_{std::random_device{}()};

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Service<kv_executive::srv::PlanPath>::SharedPtr planning_service_;
  rclcpp::Service<kv_executive::srv::PlanToPose>::SharedPtr target_planning_service_;
  rclcpp::Service<kv_executive::srv::PlanInspectionPath>::SharedPtr inspection_planning_service_;
  rclcpp::Service<kv_executive::srv::QueryInspectionGeometry>::SharedPtr
    inspection_geometry_service_;
  rclcpp::Subscription<kv_executive::msg::FireSourceReport>::SharedPtr fire_sources_subscription_;
  std::shared_ptr<const octomap::OcTree> occupancy_map_;
  mutable std::shared_mutex occupancy_map_mutex_;
  std::string occupancy_map_frame_;
  std_msgs::msg::Header occupancy_map_header_;
  std::string last_octomap_rejection_;
  std_msgs::msg::Header last_rejected_octomap_header_;
  std::deque<InspectionMap> inspection_history_;
  // Keep reception diagnostics running even when simulation time is paused.
  rclcpp::Clock octomap_clock_{RCL_STEADY_TIME};
  // Only accessed by the node's single-threaded callbacks, including the wall
  // watchdog. Duplicate map traffic cannot restart simulated clock progress.
  mutable double inspection_last_ros_clock_{std::numeric_limits<double>::quiet_NaN()};
  mutable double inspection_last_clock_progress_{0.0};
  rclcpp::Time last_octomap_received_{octomap_clock_.now()};
  rclcpp::TimerBase::SharedPtr octomap_watchdog_timer_;

  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry latest_odometry_;
  geometry_msgs::msg::PoseStamped home_pose_source_;
  geometry_msgs::msg::Pose current_pose;
  geometry_msgs::msg::Point home_pose;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_mavros_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_pos_sub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
