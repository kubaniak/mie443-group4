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
#include <cmath>

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters so that
    // MoveGroupInterface builds the SO-ARM101 model
    {
        std::string desc_dir = ament_index_cpp::get_package_share_directory("lerobot_description");
        std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
        if (urdf_file.is_open()) {
            std::stringstream ss;
            ss << urdf_file.rdbuf();
            node->declare_parameter("robot_description", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
        }

        std::string moveit_dir = ament_index_cpp::get_package_share_directory("lerobot_moveit");
        std::ifstream srdf_file(moveit_dir + "/config/so101.srdf");
        if (srdf_file.is_open()) {
            std::stringstream ss;
            ss << srdf_file.rdbuf();
            node->declare_parameter("robot_description_semantic", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm SRDF file");
        }
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Initialize helper objects
    //Navigation nav(node);
    YoloInterface yolo(node);
    ArmController arm(node);
    //AprilTagDetector apriltag(node);

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    RCLCPP_INFO(node->get_logger(), "Moving arm to pickup position");
            arm.openGripper();
            //safe carry position
            arm.moveToCartesianPose(0.174, 0.001, 0.272, 
                0.100, -0.510, -0.028, 0.854);
            arm.moveToCartesianPose(0.138, -0.021, 0.177, 
                -0.009, 0.062, -0.127, 0.990);
            if (arm.moveToCartesianPose(0.121, -0.020, 0.155, -0.006, 0.039, -0.120, 0.992)) {
                arm.closeGripper();
                RCLCPP_INFO(node->get_logger(), "Object picked up successfully.");

                // Move arm to a safe carry position (done)
                arm.moveToCartesianPose(0.138, -0.021, 0.177, 
                -0.009, 0.062, -0.127, 0.990);
                arm.moveToCartesianPose(0.174, 0.001, 0.272, 
                0.100, -0.510, -0.028, 0.854);  // home/carry position
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to move arm to pickup position!");
            }

    rclcpp::shutdown();
    return 0;
}
