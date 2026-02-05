#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// Utility functions for angle conversions
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
        // Initialize publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));

        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            std::bind(&Contest1Node::mapCallback, this, std::placeholders::_1));

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize variables
        start_time_ = this->now();
        angular_ = 0.0;
        linear_ = 0.0;
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;
        minLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;
        laserDist90Left_ = std::numeric_limits<float>::infinity();
        laserDist90Right_ = std::numeric_limits<float>::infinity();
        // temp = 0;

        // Initialize bumper states
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;
        
        // Initialize turning state
        is_turning_ = false;
        target_yaw_ = 0.0;
        turn_direction_ = 1;
        
        // Initialize bumper collision state
        is_reversing_ = false;
        reverse_duration_ = 0.5;  // Reverse for 0.5 seconds
        last_bumper_hit_ = "";
        
        // Seed random number generator
        srand(time(NULL));

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        nLasers_ = (scan->angle_max - scan->angle_min) / scan->angle_increment;
        laserRange_ = scan->ranges;
        desiredNLasers_ = deg2rad(desiredAngle_) / scan->angle_increment;
        // RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset: %d", nLasers_, desiredNLasers_);

        // Store and log distances at 90 degrees left and right
        uint32_t left_90_idx = nLasers_ / 2;
        uint32_t right_90_idx = 0;
        laserDist90Left_ = laserRange_[left_90_idx];
        laserDist90Right_ = laserRange_[right_90_idx];
        // RCLCPP_INFO(this->get_logger(), "Laser 90° Left: %.2f m, 90° Right: %.2f m", 
        //             laserDist90Left_, laserDist90Right_);

        // LIDAR has 90 degree offset, so we need to adjust indices accordingly
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;

        minLaserDist_ = std::numeric_limits<float>::infinity();

        // Find minimum laser distance in the desired angle range
        if (deg2rad(desiredAngle_) < scan->angle_max && deg2rad(desiredAngle_) > scan->angle_min)
        {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; ++laser_idx)
            {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        // RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Yaw: %.2f degrees", pos_x_, pos_y_, rad2deg(yaw_));
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        for (auto &[key, value] : bumpers_)
        {
            value = false;
        }

        for (const auto &detection : hazard_vector->detections)
        {
            // HazardDetection types: BUMP, CLIFF, STALL, WHEEL_DROP, etc.
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP)
            {
                bumpers_[detection.header.frame_id] = true;
                RCLCPP_WARN(this->get_logger(), "Bumper triggered: %s",
                            detection.header.frame_id.c_str());
            }
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
/*         // This function analyzes the map to find the least explored direction.
        // It divides the area around the robot into 8 slices (22.5 degrees each)
        // and calculates the average occupancy value for each slice within a 2-meter radius.
        // A lower average value indicates a less explored area (more unknown cells).

        const double radius = 2.0; // 2-meter radius
        const int num_slices = 16; // 8 slices of 22.5 degrees each
        std::vector<double> slice_scores(num_slices, 0.0);
        std::vector<int> slice_counts(num_slices, 0);

        double map_resolution = map->info.resolution;
        double map_origin_x = map->info.origin.position.x;
        double map_origin_y = map->info.origin.position.y;

        // Convert robot's world coordinates to map coordinates
        int robot_map_x = static_cast<int>((pos_x_ - map_origin_x) / map_resolution);
        int robot_map_y = static_cast<int>((pos_y_ - map_origin_y) / map_resolution);

        int radius_in_cells = static_cast<int>(radius / map_resolution);

        // Iterate over a square bounding box around the robot
        for (int y = robot_map_y - radius_in_cells; y <= robot_map_y + radius_in_cells; ++y)
        {
            for (int x = robot_map_x - radius_in_cells; x <= robot_map_x + radius_in_cells; ++x)
            {
                // Check if the cell is within the map boundaries
                if (x < 0 || x >= (int)map->info.width || y < 0 || y >= (int)map->info.height)
                {
                    continue;
                }

                // Check if the cell is within the circular radius
                double dist_x = (x - robot_map_x) * map_resolution;
                double dist_y = (y - robot_map_y) * map_resolution;
                if (std::sqrt(dist_x * dist_x + dist_y * dist_y) > radius)
                {
                    continue;
                }

                // Calculate angle of the cell relative to the robot's current orientation
                double angle = std::atan2(dist_y, dist_x) - yaw_;
                
                // Normalize angle to [0, 2*PI)
                while (angle < 0) angle += 2 * M_PI;
                while (angle >= 2 * M_PI) angle -= 2 * M_PI;

                // Determine which slice the cell belongs to
                int slice_index = static_cast<int>(angle / (2 * M_PI / num_slices));

                // Get occupancy value and add to the slice's score
                int map_index = y * map->info.width + x;
                slice_scores[slice_index] += map->data[map_index];
                slice_counts[slice_index]++;
            }
        }

        double min_score = std::numeric_limits<double>::max();
        int least_explored_slice = -1;

        RCLCPP_INFO(this->get_logger(), "--- Exploration Slices ---");
        for (int i = 0; i < num_slices; ++i)
        {
            double avg_score = (slice_counts[i] > 0) ? slice_scores[i] / slice_counts[i] : 100.0;
            double slice_angle_deg = (i * 360.0/num_slices) + (360.0/num_slices / 2.0);
            RCLCPP_INFO(this->get_logger(), "Slice %d (%.1f deg): Avg. Confidence = %.2f", i, slice_angle_deg, avg_score);

            if (avg_score < min_score)
            {
                min_score = avg_score;
                least_explored_slice = i;
            }
        }

        if (least_explored_slice != -1)
        {
            double best_direction_deg = (least_explored_slice * 360.0/num_slices) + (360.0/num_slices / 2.0);
            RCLCPP_INFO(this->get_logger(), "Least explored direction is slice %d (yaw-relative: %.1f deg)", least_explored_slice, best_direction_deg);
        }
        RCLCPP_INFO(this->get_logger(), "--------------------------");

        // Is this approach a good idea?
        // Yes, this is a solid foundational concept for a frontier-based exploration strategy.
        // By identifying the direction with the most unknown cells (lowest average confidence),
        // we can direct the robot to explore new areas, which is the core idea of mapping.
        //
        // Potential improvements:
        // 1. Frontier Clustering: Instead of just a direction, we could identify and cluster contiguous
        //    frontier cells to find the largest, most promising area to explore.
        // 2. Cost-Benefit Analysis: We could weigh the "reward" of exploring a frontier (its size)
        //    against the "cost" of reaching it (the distance).
        // 3. Integration with Navigation: The chosen direction needs to be translated into a goal
        //    for a navigation system (like Nav2) to drive the robot there safely.
        //
        // For this contest, this "pizza slice" method is a clever and efficient way to make
        // an initial exploration decision without the complexity of a full frontier-based planner.
 */    }
    void controlLoop()
    {
         // Calculate elapsed time
        auto current_time = this->now();
        double seconds_elapsed = (current_time - start_time_).seconds();

        // Check if 480 seconds (8 minutes) have elapsed
        if (seconds_elapsed >= 480.0)
        {
            RCLCPP_INFO(this->get_logger(), "Contest time completed (480 seconds). Stopping robot.");

            // Stop the robot
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;
            vel_pub_->publish(vel);

            // Shutdown the node
            rclcpp::shutdown();
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %.2f rad or %.2f deg, Min LIDAR Dist: %.2f m",
        //             pos_x_, pos_y_, yaw_, rad2deg(yaw_), minLaserDist_);

        // Check for bumper collision first
        bool any_bumper_pressed = false;
        for (const auto &[key, value] : bumpers_)
        {
            if (value)
            {
                any_bumper_pressed = true;
                break;
            }
        }
        
        // Exploration logic: bumper override, then turning, then obstacle avoidance, then forward
        if (any_bumper_pressed && !is_reversing_)
        {
            // Bumper hit! Start reversing and store which bumper was hit
            is_reversing_ = true;
            reverse_start_time_ = this->now();
            angular_ = 0.0;
            linear_ = -0.15;  // Reverse at 0.15 m/s
            is_turning_ = false;  // Cancel any ongoing turn
            
            // Store which bumper was hit
            if (bumpers_["bump_front_left"] || bumpers_["bump_left"])
                last_bumper_hit_ = "left";
            else if (bumpers_["bump_front_right"] || bumpers_["bump_right"])
                last_bumper_hit_ = "right";
            else if (bumpers_["bump_front_center"])
                last_bumper_hit_ = "center";
            else
                last_bumper_hit_ = "unknown";
            
            RCLCPP_WARN(this->get_logger(), "Bumper collision (%s)! Reversing...", last_bumper_hit_.c_str());
        }
        else if (is_reversing_)
        {
            // Currently reversing - check if duration elapsed
            double reverse_elapsed = (this->now() - reverse_start_time_).seconds();
            
            if (reverse_elapsed >= reverse_duration_)
            {
                // Finished reversing - start turning
                is_reversing_ = false;
                
                // Determine turn direction based on which bumper was hit
                if (last_bumper_hit_ == "left")
                    turn_direction_ = -1;  // Turn right (away from left obstacle)
                else if (last_bumper_hit_ == "right")
                    turn_direction_ = 1;   // Turn left (away from right obstacle)
                else if (last_bumper_hit_ == "center")
                {
                    // Center bumper - use laser obstacle detection logic
                    if (laserDist90Left_ < laserDist90Right_)
                        turn_direction_ = -1;  // Turn right
                    else
                        turn_direction_ = 1;   // Turn left
                }
                else
                    turn_direction_ = (rand() % 2 == 0) ? 1 : -1;  // Fallback to random
                
                target_yaw_ = yaw_ + turn_direction_ * deg2rad(30.0 + (rand() % 91));  // Random angle between 30-120 degrees
                
                // Normalize target yaw to [-pi, pi]
                while (target_yaw_ > M_PI) target_yaw_ -= 2 * M_PI;
                while (target_yaw_ < -M_PI) target_yaw_ += 2 * M_PI;
                
                is_turning_ = true;
                angular_ = turn_direction_ * M_PI / 4;
                linear_ = 0.0;
                
                RCLCPP_INFO(this->get_logger(), "Reverse complete. Bumper hit: %s. Turning %s.",
                           last_bumper_hit_.c_str(), turn_direction_ > 0 ? "left" : "right");
            }
            else
            {
                // Continue reversing
                angular_ = 0.0;
                linear_ = -0.15;
            }
        }
        else if (is_turning_)
        {
            // Currently turning - check if we've reached target yaw
            double yaw_diff = target_yaw_ - yaw_;
            
            // Normalize angle difference to [-pi, pi]
            while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
            
            if (std::abs(yaw_diff) < 0.1)  // Within tolerance
            {
                // Finished turning
                is_turning_ = false;
                angular_ = 0.0;
                linear_ = 0.2;  // Resume forward movement
                RCLCPP_INFO(this->get_logger(), "Turn complete. Resuming forward motion.");
            }
            else
            {
                // Continue turning
                angular_ = turn_direction_ * M_PI / 4;  // Turn at 45 deg/s
                linear_ = 0.0;
            }
        }
        else if (minLaserDist_ < 0.5)
        {
            // Obstacle detected - decide turn direction
            if (laserDist90Left_ < laserDist90Right_)
                turn_direction_ = -1;  // Turn right
            else
                turn_direction_ = 1;   // Turn left
            
            target_yaw_ = yaw_ + turn_direction_ * deg2rad(30.0 + (rand() % 91));  // Random angle between 30-120 degrees

            // Normalize target yaw to [-pi, pi]
            while (target_yaw_ > M_PI) target_yaw_ -= 2 * M_PI;
            while (target_yaw_ < -M_PI) target_yaw_ += 2 * M_PI;
            
            is_turning_ = true;
            angular_ = turn_direction_ * M_PI / 4;  // Start turning at 45 deg/s
            linear_ = 0.0;

            RCLCPP_INFO(this->get_logger(), "Obstacle detected at %.2f m! Turning %s by %.1f degrees.",
                       minLaserDist_, turn_direction_ > 0 ? "left" : "right", rad2deg(target_yaw_ - yaw_));
        }
        else
        {
            // No obstacle - move forward
            angular_ = 0.0;
            linear_ = 0.2;
        }

        // Set velocity command
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;

        // Publish velocity command
        vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    float angular_;
    float linear_;
    double pos_x_;
    double pos_y_;
    double yaw_;
    std::map<std::string, bool> bumpers_;
    float minLaserDist_;
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;
    float laserDist90Left_;
    float laserDist90Right_;
    
    // State variables for turning behavior
    bool is_turning_;
    double target_yaw_;
    int turn_direction_;
    
    // State variables for bumper collision handling
    bool is_reversing_;
    rclcpp::Time reverse_start_time_;
    double reverse_duration_;  // How long to reverse (seconds)
    std::string last_bumper_hit_;  // Store which bumper was hit
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
