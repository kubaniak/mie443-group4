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

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // Initialize box coordinates
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }
    
    // Initialize YOLO object detector
    YoloInterface yoloDetector(node);

    ArmController armController(node);

    // TEST CODE FOR ARM CONTROL
    // RCLCPP_INFO(node->get_logger(), "TESTING ARM CONTROL");
    // bool success = armController.moveToCartesianPose(0.043, 0.199, 0.313,
    //                                                 -0.471, -0.557, 0.564, -0.387);
    // if (success) {
    //     RCLCPP_INFO(node->get_logger(), "Successfully moved to Cartesian pose");
    //     // Test gripper
    //     RCLCPP_INFO(node->get_logger(), "Testing gripper control");
    //     armController.openGripper();
    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    //     armController.closeGripper();
    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move to Cartesian pose");
    // }

    // success = armController.moveToCartesianPose(0.142, -0.064, 0.400, 
    //                                             -0.418, 0.844, 0.238, -0.237);
    // if (success) {
    //     RCLCPP_INFO(node->get_logger(), "Successfully moved to Cartesian pose");
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move to Cartesian pose");
    // }
    
    // if (success) {
    //     RCLCPP_INFO(node->get_logger(), "Successfully moved to Cartesian pose");
    //     // Test gripper
    //     RCLCPP_INFO(node->get_logger(), "Testing gripper control");
    //     armController.openGripper();
    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    //     armController.closeGripper();
    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move to Cartesian pose");
    // }

    // success = armController.moveToCartesianPose(0.142, -0.064, 0.400, 
    //                                             -0.418, 0.844, 0.238, -0.237);
    // if (success) {
    //     RCLCPP_INFO(node->get_logger(), "Successfully moved to Cartesian pose");
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move to Cartesian pose");
    // }


    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    RCLCPP_INFO(node->get_logger(), "Moving Arm for selfie...");

    // From Simulation: 
    // Translation: [0.159, 0.025, 0.172]
    // Rotation: in Quaternion (xyzw) [0.109, 0.153, 0.801, 0.568]
    
    bool success = armController.moveToCartesianPose(0.159, 0.025, 0.172, 
                                                     0.109, 0.153, 0.801, 0.568);
    if (success) {
        RCLCPP_INFO(node->get_logger(), "Successfully moved to Cartesian pose for selfie");
        // Take one picture of 
        RCLCPP_INFO(node->get_logger(), "Attempting YOLO (WRIST Camera) detection at %lu seconds", secondsElapsed);
        std::string detected = yoloDetector.getObjectName(CameraSource::WRIST, true);

        if (!detected.empty()) {
            float confidence = yoloDetector.getConfidence();
            RCLCPP_INFO(node->get_logger(), "YOLO detected: %s with confidence %.2f", detected.c_str(), confidence);
        } else {
            RCLCPP_INFO(node->get_logger(), "YOLO did not detect any objects");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to Cartesian pose for selfie");
    }

    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();                
        
        // // TEST CODE FOR YOLO DETECTION
        // static uint64_t lastYoloTime = 0;
        // if (secondsElapsed >= lastYoloTime + 2) { 
        //     lastYoloTime = secondsElapsed; // Update last YOLO time to current time
        //     RCLCPP_INFO(node->get_logger(), "Attempting YOLO (WRIST Camera) detection at %lu seconds", secondsElapsed);
        //     std::string detected = yoloDetector.getObjectName(CameraSource::WRIST, true);

        //     if (!detected.empty()) {
        //         float confidence = yoloDetector.getConfidence();
        //         RCLCPP_INFO(node->get_logger(), "YOLO detected: %s with confidence %.2f", detected.c_str(), confidence);
        //     } else {
        //         RCLCPP_INFO(node->get_logger(), "YOLO did not detect any objects");
        //     }
        // }

        /***YOUR CODE HERE***/
        // Localization: spin the robot for localization at the start of the contest TODO: Change seconds
        if (secondsElapsed < 4) {
            // Spin in place for localization
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = node->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.5;  // Spin at 0.5 rad/s
            vel_pub->publish(vel);
            continue;  // Skip the rest of the loop while localizing
        } else if (secondsElapsed >= 4 && secondsElapsed < 6) {
            // Stop the robot after spinning
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = node->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;  // Stop spinning
            vel_pub->publish(vel);
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
