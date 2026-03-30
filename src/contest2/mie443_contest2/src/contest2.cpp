#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>


int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    auto vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    // Initialize AprilTag detector in camera optical frame (camera -> tag transform).
    AprilTagDetector aprilTagDetector(node);
    aprilTagDetector.setReferenceFrame("oakd_rgb_camera_optical_frame");

    std::vector<int> candidate_tags = {0, 1, 2, 3, 4};

    // Desired camera->tag alignment.
    geometry_msgs::msg::Vector3 t_des;
    t_des.x = node->declare_parameter<double>("target_x", -0.009);
    t_des.y = node->declare_parameter<double>("target_y", 0.034);
    t_des.z = node->declare_parameter<double>("target_z", 0.196);

    geometry_msgs::msg::Quaternion q_des;
    q_des.x = node->declare_parameter<double>("target_qx", 1.000);
    q_des.y = node->declare_parameter<double>("target_qy", 0.008);
    q_des.z = node->declare_parameter<double>("target_qz", 0.015);
    q_des.w = node->declare_parameter<double>("target_qw", -0.002);

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    bool aligned_to_tag = false;
    int aligned_tag_id = -1;

    while(rclcpp::ok() && secondsElapsed <= 300 && !aligned_to_tag) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();                

        // TAG ALIGNMENT CODE HERE
        auto visible_tags = aprilTagDetector.getVisibleTags(candidate_tags, 50);

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node->now();
        cmd_vel.header.frame_id = "base_link";

        if (visible_tags.empty()) {
            // Drive in a small circle while searching for a tag.
            cmd_vel.twist.linear.x = 0.06;
            cmd_vel.twist.angular.z = 0.28;
            vel_pub->publish(cmd_vel);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        int tag_id = visible_tags.front();
        auto tag_tf_opt = aprilTagDetector.getTagTransform(tag_id, 50);
        if (!tag_tf_opt.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        const auto & t = tag_tf_opt->transform.translation;
        const double x_error = t.x - t_des.x;  // Horizontal tag offset in camera frame.
        const double z_error = t.z - t_des.z;  // Forward distance error.

        const double k_angular = 1.8;
        const double k_linear = 0.8;
        const double max_angular = 0.7;
        const double max_linear = 0.2;
        const double x_tolerance = 0.015;
        const double z_tolerance = 0.03;

        // In camera optical frame, +x means tag appears to the right.
        cmd_vel.twist.angular.z = std::clamp(-k_angular * x_error, -max_angular, max_angular);
        cmd_vel.twist.linear.x = std::clamp(k_linear * z_error, -max_linear, max_linear);

        vel_pub->publish(cmd_vel);

        if (std::abs(x_error) < x_tolerance && std::abs(z_error) < z_tolerance) {
            aligned_to_tag = true;
            aligned_tag_id = tag_id;

            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "base_link";
            vel_pub->publish(stop_cmd);

            RCLCPP_INFO(node->get_logger(),
                        "Aligned to AprilTag %d (x_error=%.3f, z_error=%.3f)",
                        aligned_tag_id, x_error, z_error);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (aligned_to_tag) {
        RCLCPP_INFO(node->get_logger(), "AprilTag alignment complete.");
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
