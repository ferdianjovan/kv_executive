#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <mutex>
#include <shared_mutex>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class MotionPlanner : public rclcpp::Node
{
public:
    MotionPlanner(): Node("motion_planner")
    {
        loadPlannerParameters();

        auto rpose_cb = [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
            current_pose = msg->pose;
            RCLCPP_INFO(
                this->get_logger(), "Current pose x: %.2f, y: %.2f, z: %.2f",
                current_pose.position.x, current_pose.position.y, current_pose.position.z
            );
            current_pose_received_ = true;
        };
        rpose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(), rpose_cb);

        auto hpose_cb = [this](mavros_msgs::msg::HomePosition::SharedPtr msg) -> void {
            home_pose = msg->position;
            RCLCPP_INFO(
                this->get_logger(), "Home pose x: %.2f, y: %.2f, z: %.2f",
                home_pose.x, home_pose.y, home_pose.z
            );
            home_pose_received_ = true;
        };
        home_pos_sub = this->create_subscription<mavros_msgs::msg::HomePosition>(
            "/mavros/home_position/home", rclcpp::SensorDataQoS(), hpose_cb);

        param_mavros_ = this->create_client<rcl_interfaces::srv::GetParameters>(
            "/mavros/param/get_parameters");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&MotionPlanner::requestMaximumAltitude, this));

        octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            std::bind(&MotionPlanner::octomapCallback, this, std::placeholders::_1)
        );
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "planned_path", rclcpp::QoS(1).reliable().transient_local());

        createStateSpace();
        planning_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000), std::bind(&MotionPlanner::planPath, this));
    }

    void createPlanningSetup(){
        ss_ = std::make_shared<og::SimpleSetup>(space_);
        ss_->setStateValidityChecker([this](const ob::State *state){ return isStateValid(state); });
        const auto si = ss_->getSpaceInformation();
        si->setStateValidityCheckingResolution(validity_resolution_fraction_);
        auto planner = std::make_shared<og::RRTConnect>(si);
        ss_->setPlanner(planner);
    }

    bool planPath(){
        auto goal_position = randomWalkGoal();
        return createPath(goal_position);
    }

    geometry_msgs::msg::Point randomWalkGoal(){
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

        geometry_msgs::msg::Point goal_position;
        goal_position.x =
            std::uniform_real_distribution<double>(min_x, max_x)(random_generator_);
        goal_position.y =
            std::uniform_real_distribution<double>(min_y, max_y)(random_generator_);

        // updatePositionSpaceBounds() will report the invalid altitude range.
        // Avoid constructing a distribution with reversed limits in that case.
        goal_position.z = min_z <= max_z
            ? std::uniform_real_distribution<double>(min_z, max_z)(random_generator_)
            : current_pose.position.z;

        return goal_position;
    }

    bool createPath(const geometry_msgs::msg::Point &goal_position){
        if (!current_pose_received_ || !home_pose_received_ || !occupancy_map_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot create path: required data are unavailable");
            return false;
        }

        if (!updatePositionSpaceBounds(goal_position)){
            return false;
        }

        createPlanningSetup();
        // create a start state
        ob::ScopedState<> start(space_);
        auto *compound_state = start->as<ob::CompoundStateSpace::StateType>();
        auto *position_state = compound_state->as<ob::RealVectorStateSpace::StateType>(0);
        position_state->values[0] = current_pose.position.x;
        position_state->values[1] = current_pose.position.y;
        position_state->values[2] = current_pose.position.z;
        auto *yaw_state = compound_state->as<ob::SO2StateSpace::StateType>(1);
        yaw_state->value = tf2::getYaw(current_pose.orientation);

        ob::ScopedState<> goal(space_);
        auto *goal_compound_state = goal->as<ob::CompoundStateSpace::StateType>();
        auto *goal_position_state =
            goal_compound_state->as<ob::RealVectorStateSpace::StateType>(0);
        goal_position_state->values[0] = goal_position.x;
        goal_position_state->values[1] = goal_position.y;
        goal_position_state->values[2] = goal_position.z;
        auto *goal_yaw_state = goal_compound_state->as<ob::SO2StateSpace::StateType>(1);
        goal_yaw_state->value = tf2::getYaw(current_pose.orientation);

        const auto space_information = ss_->getSpaceInformation();
        if (!space_information->isValid(start.get()))
        {
            RCLCPP_ERROR(this->get_logger(), "OMPL start state is invalid");
            return false;
        }
        if (!space_information->isValid(goal.get()))
        {
            RCLCPP_ERROR(this->get_logger(), "OMPL goal state is invalid");
            return false;
        }
        ss_->setStartAndGoalStates(start, goal);

        const auto solved = ss_->solve(planning_timeout_);
        if (!solved)
        {
            RCLCPP_WARN(this->get_logger(), "No OMPL solution found");
            return false;
        }
        ss_->simplifySolution(simplification_time_);
        auto &solution_path = ss_->getSolutionPath();

        solution_poses_ = processSolutionPath(solution_path);

        return true;
    }

    std::vector<geometry_msgs::msg::Pose> processSolutionPath(
        const og::PathGeometric &solution_path)
    {
        std::vector<geometry_msgs::msg::Pose> poses;
        poses.reserve(solution_path.getStateCount());

        nav_msgs::msg::Path path_message;
        path_message.header.stamp = this->now();
        {
            std::shared_lock<std::shared_mutex> lock(occupancy_map_mutex_);
            path_message.header.frame_id = occupancy_map_frame_;
        }

        if (path_message.header.frame_id.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Cannot publish the planned path because the OctoMap frame is empty");
        }

        path_message.poses.reserve(solution_path.getStateCount());
        for (std::size_t index = 0; index < solution_path.getStateCount(); ++index)
        {
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
            poses.push_back(pose);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_message.header;
            pose_stamped.pose = pose;
            path_message.poses.push_back(std::move(pose_stamped));
        }

        if (!path_message.header.frame_id.empty())
        {
            path_publisher_->publish(path_message);
            RCLCPP_INFO(
                this->get_logger(), "Published planned path with %zu poses in frame '%s'",
                path_message.poses.size(), path_message.header.frame_id.c_str());
        }

        return poses;
    }

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
        if (!abstract_tree)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize OctoMap message");
            return;
        }

        auto *octree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
        if (!octree)
        {
            RCLCPP_ERROR(this->get_logger(), "Received unsupported OctoMap tree type: %s", msg->id.c_str());
            return;
        }

        std::shared_ptr<const octomap::OcTree> new_map(static_cast<octomap::OcTree *>(abstract_tree.release()));
        {
            std::unique_lock<std::shared_mutex> lock(occupancy_map_mutex_);
            occupancy_map_ = std::move(new_map);
            occupancy_map_frame_ = msg->header.frame_id;
            occupancy_map_received_ = true;
        }

        RCLCPP_INFO(this->get_logger(), "Received binary OctoMap: frame=%s, resolution=%.3f m",
            msg->header.frame_id.c_str(), msg->resolution);
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

        if (horizontal_range_ <= 0.0 || vertical_range_ <= 0.0)
            throw std::invalid_argument("horizontal_range and vertical_range must be greater than zero");
        if (minimum_altitude_ >= maximum_altitude_)
            throw std::invalid_argument("minimum_altitude must be less than maximum_altitude");
        if (bound_margin_ < 0.0 || drone_radius_ < 0.0 || obstacle_clearance_ < 0.0)
            throw std::invalid_argument("bound_margin, drone_radius, and obstacle_clearance cannot be negative");
        if (yaw_weight_ <= 0.0)
            throw std::invalid_argument("yaw_weight must be greater than zero");
        if (validity_resolution_fraction_ <= 0.0 || validity_resolution_fraction_ > 1.0)
            throw std::invalid_argument("validity_resolution_fraction must be in the range (0, 1]");
        if (planning_timeout_ <= 0.0)
            throw std::invalid_argument("planning_timeout must be greater than zero");
        if (simplification_time_ <= 0.0)
            throw std::invalid_argument("simplification_time must be greater than zero");
    }

    void createStateSpace(){
        position_space_ = std::make_shared<ob::RealVectorStateSpace>(3);
        yaw_space_ = std::make_shared<ob::SO2StateSpace>();

        space_ = std::make_shared<ob::CompoundStateSpace>();
        space_->addSubspace(position_space_, 1.0);
        space_->addSubspace(yaw_space_, yaw_weight_);
        // Prevent any more subspaces being added later.
        space_->lock();
    }

    bool updatePositionSpaceBounds(const geometry_msgs::msg::Point &goal_position)
    {
        const double drone_x = current_pose.position.x;
        const double drone_y = current_pose.position.y;
        const double drone_z = current_pose.position.z;

        const double home_z = home_pose.z;

        const double min_x = drone_x - horizontal_range_;
        const double max_x = drone_x + horizontal_range_;
        const double min_y = drone_y - horizontal_range_;
        const double max_y = drone_y + horizontal_range_;
        const double min_z = std::max(home_z + minimum_altitude_ + bound_margin_, drone_z - vertical_range_);
        const double max_z = std::min(home_z + maximum_altitude_ - bound_margin_, drone_z + vertical_range_);

        if (min_x >= max_x || min_y >= max_y || min_z >= max_z)
        {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid planning bounds: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f]",
                min_x, max_x, min_y, max_y, min_z, max_z
            );
            return false;
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

        const bool goal_inside_bounds = goal_position.x >= bounds.low[0] && goal_position.x <= bounds.high[0] &&
            goal_position.y >= bounds.low[1] && goal_position.y <= bounds.high[1] &&
            goal_position.z >= bounds.low[2] && goal_position.z <= bounds.high[2];
        if (!goal_inside_bounds){
            RCLCPP_WARN(this->get_logger(), "Goal is outside the current planning bounds");
            return false;
        }
        return true;
    }

    bool isStateValid(const ob::State *state) const
    {
        // 1. State must lie inside the rolling planning bounds.
        if (!space_->satisfiesBounds(state))
        {
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
        const std::shared_ptr<const octomap::OcTree> &occupancy_map,
        double x, double y, double z, double radius) const
    {
        if (!occupancy_map || radius < 0.0)
            return false;

        const octomap::point3d minimum(x - radius, y - radius, z - radius);
        const octomap::point3d maximum(x + radius, y + radius, z + radius);
        const double radius_squared = radius * radius;

        for (auto iterator = occupancy_map->begin_leafs_bbx(minimum, maximum),
                  end = occupancy_map->end_leafs_bbx(); iterator != end; ++iterator)
        {
            if (!occupancy_map->isNodeOccupied(*iterator))
                continue;

            // Find the squared distance from the sphere center to the closest
            // point in this leaf voxel's axis-aligned bounding box.
            const double half_size = iterator.getSize() * 0.5;
            const double dx = std::max(std::abs(x - iterator.getX()) - half_size, 0.0);
            const double dy = std::max(std::abs(y - iterator.getY()) - half_size, 0.0);
            const double dz = std::max(std::abs(z - iterator.getZ()) - half_size, 0.0);

            if (dx * dx + dy * dy + dz * dz <= radius_squared)
                return true;
        }
        return false;
    }

    void requestMaximumAltitude()
    {
        if (!param_mavros_->service_is_ready()){
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000, "Waiting for MAVROS parameter service..."
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
                if (response->values.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "MAVROS returned no value for MAXIMUM ALTITUDE");
                    return;
                }
                const auto &value = response->values.front();
                if(value.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                {
                    RCLCPP_ERROR(this->get_logger(), "MAXIMUM ALTITUDE has unexpected ROS parameter type");
                    return;
                }
                const double max_altitude = value.double_value;
                if (max_altitude <= 0.0){
                    RCLCPP_WARN(this->get_logger(), "MAXIMUM ALTITUDE is disabled, using default MAXIMUM ALTITUDE");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Robot maximum vertical distance from Home: %.2f m", max_altitude);
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
    bool current_pose_received_{false};
    bool home_pose_received_{false};
    bool occupancy_map_received_{false};
    bool fire_position_received_{false};

    std::shared_ptr<ob::RealVectorStateSpace> position_space_;
    std::shared_ptr<ob::SO2StateSpace> yaw_space_;
    std::shared_ptr<ob::CompoundStateSpace> space_;
    std::shared_ptr<og::SimpleSetup> ss_;
    std::mt19937 random_generator_{std::random_device{}()};
    std::vector<geometry_msgs::msg::Pose> solution_poses_;

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    std::shared_ptr<const octomap::OcTree> occupancy_map_;
    mutable std::shared_mutex occupancy_map_mutex_;
    std::string occupancy_map_frame_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr planning_timer_;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Point home_pose;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_mavros_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rpose_sub;
    rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_pos_sub;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
