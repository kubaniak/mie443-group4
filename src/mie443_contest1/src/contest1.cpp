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
        
        // Wall-following parameters
        desired_wall_distance_ = 0.6;  // Target distance from right wall (meters)
        wall_following_gain_ = 0.8;     // Proportional gain for steering correction
        wall_detect_threshold_ = 1.0;   // Max distance to consider a wall present
        right_wall_distance_ = std::numeric_limits<float>::infinity();

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
        
        // Calculate average distance to right wall (from 20° to 70° on the right side)
        // Right side is at lower indices (0 is rightmost)
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;
        uint32_t right_start_angle = deg2rad(20.0);  // 20 degrees to the right
        uint32_t right_end_angle = deg2rad(70.0);    // 70 degrees to the right
        uint32_t right_start_idx = (right_start_angle / scan->angle_increment);
        uint32_t right_end_idx = (right_end_angle / scan->angle_increment);
        
        right_wall_distance_ = std::numeric_limits<float>::infinity();
        int valid_readings = 0;
        float sum_distance = 0.0;
        
        for (uint32_t idx = right_start_idx; idx <= right_end_idx && idx < nLasers_; ++idx) {
            if (std::isfinite(laserRange_[idx]) && laserRange_[idx] > 0.1) {
                sum_distance += laserRange_[idx];
                valid_readings++;
            }
        }
        
        if (valid_readings > 0) {
            right_wall_distance_ = sum_distance / valid_readings;
        }

        // LIDAR has 90 degree offset, so we need to adjust indices accordingly
        laser_offset = deg2rad(-90.0);
        front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;

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
            if (bumpers_["bump_left"])
                last_bumper_hit_ = "side_left";
            else if (bumpers_["bump_right"])
                last_bumper_hit_ = "side_right";
            else if (bumpers_["bump_front_left"])
                last_bumper_hit_ = "front_left";
            else if (bumpers_["bump_front_right"])
                last_bumper_hit_ = "front_right";
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
                
                // Determine turn direction and angle based on which bumper was hit
                double turn_angle_deg;
                
                if (last_bumper_hit_ == "side_left")
                {
                    turn_direction_ = -1;  // Turn right (away from left obstacle)
                    turn_angle_deg = 5.0 + (rand() % 11);  // Random angle between 5-15 degrees
                }
                else if (last_bumper_hit_ == "side_right")
                {
                    turn_direction_ = 1;   // Turn left (away from right obstacle)
                    turn_angle_deg = 5.0 + (rand() % 11);  // Random angle between 5-15 degrees
                }
                else if (last_bumper_hit_ == "front_left")
                {
                    turn_direction_ = -1;  // Turn right (away from left obstacle)
                    turn_angle_deg = 30.0 + (rand() % 91);  // Random angle between 30-120 degrees
                }
                else if (last_bumper_hit_ == "front_right")
                {
                    turn_direction_ = 1;   // Turn left (away from right obstacle)
                    turn_angle_deg = 30.0 + (rand() % 91);  // Random angle between 30-120 degrees
                }
                else if (last_bumper_hit_ == "center")
                {
                    // Center bumper - use laser obstacle detection logic
                    if (laserDist90Left_ < laserDist90Right_)
                        turn_direction_ = -1;  // Turn right
                    else
                        turn_direction_ = 1;   // Turn left
                    turn_angle_deg = 30.0 + (rand() % 91);  // Random angle between 30-120 degrees
                }
                else
                {
                    turn_direction_ = (rand() % 2 == 0) ? 1 : -1;  // Fallback to random
                    turn_angle_deg = 30.0 + (rand() % 91);  // Random angle between 30-120 degrees
                }
                
                target_yaw_ = yaw_ + turn_direction_ * deg2rad(turn_angle_deg);
                
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
            // No obstacle - move forward with wall following if wall detected
            linear_ = 0.2;
            
            // Check if we have a wall on the right side
            if (right_wall_distance_ < wall_detect_threshold_)
            {
                // Wall detected! Apply wall-following steering correction
                float distance_error = right_wall_distance_ - desired_wall_distance_;
                
                // Proportional control: if too close to wall (error < 0), turn left (positive angular)
                // if too far from wall (error > 0), turn right (negative angular)
                angular_ = -wall_following_gain_ * distance_error;
                
                // Clamp angular velocity to reasonable limits
                float max_angular = 0.5;  // rad/s
                if (angular_ > max_angular) angular_ = max_angular;
                if (angular_ < -max_angular) angular_ = -max_angular;
                
                // RCLCPP_INFO(this->get_logger(), "Wall following: distance=%.2f m, error=%.2f, angular=%.2f",
                //            right_wall_distance_, distance_error, angular_);
            }
            else
            {
                // No wall detected - go straight (existing behavior)
                angular_ = 0.0;
            }
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
    
    // Wall-following variables
    float desired_wall_distance_;    // Target distance to maintain from right wall
    float wall_following_gain_;      // Proportional gain for steering correction
    float wall_detect_threshold_;    // Maximum distance to consider a wall present
    float right_wall_distance_;      // Current average distance to right wall
    
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
