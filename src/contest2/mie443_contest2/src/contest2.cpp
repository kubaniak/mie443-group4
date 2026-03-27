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
    // Position just above the cup
    arm.moveToCartesianPose(0.142, -0.019, 0.181,
        -0.016, 0.108, -0.114, 0.987);

    // Move down to grasp the cup
    arm.moveToCartesianPose(0.126, -0.018, 0.152,
        0.001, -0.009, -0.110, 0.994);

    arm.closeGripper();

    // Position just above the cup
    arm.moveToCartesianPose(0.142, -0.019, 0.181,
        -0.016, 0.108, -0.114, 0.987);

    // Intermediate safe position
    arm.moveToCartesianPose(0.028, -0.145, 0.139, 
        -0.289, -0.407, -0.674, 0.544);

    // Move to a position where the OAKD camera can see the cup
    arm.moveToCartesianPose(0.018, -0.311, 0.097, 
        -0.073, 0.049, 0.544, 0.835);  
    
    
    // // TEST CODE FOR YOLO DETECTION
    // while(rclcpp::ok() && secondsElapsed <= 300) {
    //     rclcpp::spin_some(node);

    //     // Calculate elapsed time
    //     auto now = std::chrono::system_clock::now();
    //     secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        
    //     // TEST CODE FOR YOLO DETECTION
    //     static uint64_t lastYoloTime = 0;
    //     if (secondsElapsed >= lastYoloTime + 2) { 
    //         lastYoloTime = secondsElapsed; // Update last YOLO time to current time
    //         RCLCPP_INFO(node->get_logger(), "Attempting YOLO (OAKD Camera) detection at %lu seconds", secondsElapsed);
    //         std::string detected = yolo.getObjectName(CameraSource::OAKD, true);

    //         if (!detected.empty()) {
    //             float confidence = yolo.getConfidence();
    //             RCLCPP_INFO(node->get_logger(), "YOLO detected: %s with confidence %.2f", detected.c_str(), confidence);
    //         } else {
    //             RCLCPP_INFO(node->get_logger(), "YOLO did not detect any objects");
    //         }
    //     }
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    
    // Home Position (also drop position!)
    arm.moveToCartesianPose(0.023, -0.277, 0.246, 
        -0.464, -0.474, -0.529, 0.529);

    arm.openGripper();

    rclcpp::shutdown();
    return 0;
}
