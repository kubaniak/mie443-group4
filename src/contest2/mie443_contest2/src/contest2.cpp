#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Initialize YOLO object detector
    AprilTagDetector aprilTagDetector(node);
    std::vector<int> candidate_tags = {0, 1, 2, 3, 4};

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();                

        static uint64_t last_printed_second = 0;
        if (secondsElapsed > last_printed_second) {
            last_printed_second = secondsElapsed;
            auto visible_tags = aprilTagDetector.getVisibleTags(candidate_tags);
            if (!visible_tags.empty()) {
                for (int tag_id : visible_tags) {
                    auto pose = aprilTagDetector.getTagPose(tag_id);
                    if (pose.has_value()) {
                        RCLCPP_INFO(node->get_logger(), "Visible tag %d at pos (%.2f, %.2f, %.2f) and or (%.2f, %.2f, %.2f, %.2f)", 
                                    tag_id, 
                                    pose->position.x, pose->position.y, pose->position.z,
                                    pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);
                    }
                }
            } else {
                RCLCPP_INFO(node->get_logger(), "No AprilTags visible at %lu seconds", secondsElapsed);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
