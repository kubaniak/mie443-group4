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
#include <unordered_set>
#include <vector>

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

    // Initialize helper objects
    // Navigation nav(node);
    // YoloInterface yolo(node);
    ArmController arm(node);
    // AprilTagDetector apriltag(node);

    std::ofstream outfile("src/contest2/mie443_contest2/contest2_output.txt", std::ios_base::app);
    if (!outfile.is_open()) {
        RCLCPP_WARN(node->get_logger(), "Could not open contest2_output.txt for writing");
    }

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;



    // Shared cmd_vel publisher used during initial spin, scan retries, and alignment.
    auto vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    // Spin 360 degrees in place so AMCL can converge before we record the start pose.
    // The TurtleBot4 expects TwistStamped on /cmd_vel.
   

    double start_x = robotPose.x;
    double start_y = robotPose.y;
    double start_phi = robotPose.phi;
    RCLCPP_INFO(node->get_logger(), "Recorded start pose: x=%.2f, y=%.2f, phi=%.2f", start_x, start_y, start_phi);

    // State variables for contest strategy
    bool pickup_done = false;
    bool object_placed = false;
    int current_box_idx = 0;
    bool return_home = false;
    bool aligned_to_tag = false; 
    std::string manipulable_object = "cup";
    bool all_boxes_visited = false;
    bool manipulate = true;

    // OAK-D scene objects are single-use: once detected confidently, we stop looking for that class.
    const std::vector<std::string> scene_class_order = {
        "cup", "motorcycle", "clock", "plant", "water bottle"
    };
    std::unordered_set<std::string> remaining_scene_objects(
        scene_class_order.begin(), scene_class_order.end());

    auto format_remaining_scene_objects = [&]() -> std::string {
        std::ostringstream oss;
        bool first = true;
        for (const auto& class_name : scene_class_order) {
            if (remaining_scene_objects.find(class_name) == remaining_scene_objects.end()) {
                continue;
            }
            if (!first) {
                oss << ", ";
            }
            oss << class_name;
            first = false;
        }
        return first ? "none" : oss.str();
    };

    auto process_scene_detection = [&](const std::string& scene_object,
                                       float confidence,
                                       const std::string& context_label,
                                       double log_x,
                                       double log_y,
                                       double log_phi,
                                       float min_detection_confidence) -> bool {
        if (scene_object.empty() || confidence <= min_detection_confidence) {
            return false;
        }

        if (remaining_scene_objects.find(scene_object) == remaining_scene_objects.end()) {
            RCLCPP_INFO(node->get_logger(),
                        "Ignoring already checked-off object '%s' (%.2f) during %s.",
                        scene_object.c_str(), confidence, context_label.c_str());
            if (outfile.is_open()) {
                outfile << "Box " << current_box_idx << " at ("
                        << log_x << ", " << log_y << ", " << log_phi << "): "
                        << "IGNORED checked-off object " << scene_object
                        << " (confidence: " << confidence << ", " << context_label << ")" << std::endl;
            }
            return false;
        }

        remaining_scene_objects.erase(scene_object);
        RCLCPP_INFO(node->get_logger(),
                    "Detected new scene object: %s (%.2f) during %s. Remaining: [%s]",
                    scene_object.c_str(), confidence, context_label.c_str(),
                    format_remaining_scene_objects().c_str());

        if (outfile.is_open()) {
            outfile << "Box " << current_box_idx << " at ("
                    << log_x << ", " << log_y << ", " << log_phi << "): "
                    << scene_object << " (confidence: " << confidence << ", " << context_label << ")"
                    << std::endl;
            outfile << "CHECKED_OFF: " << scene_object
                    << " | Remaining: [" << format_remaining_scene_objects() << "]" << std::endl;
        }

        if (scene_object == manipulable_object && !object_placed) {
            manipulate = true;
            RCLCPP_INFO(node->get_logger(), "Manipulable object '%s' detected! Will attempt to place in bin at box %d.",
                        manipulable_object.c_str(), current_box_idx);
        }
        return true;
    };

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 20) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        
        // // TEST CODE FOR YOLO DETECTION
        // static uint64_t lastYoloTime = 0;
        // if (secondsElapsed >= lastYoloTime + 2) { 
        //     lastYoloTime = secondsElapsed; // Update last YOLO time to current time
        //     RCLCPP_INFO(node->get_logger(), "Attempting YOLO (WRIST Camera) detection at %lu seconds", secondsElapsed);
        //     std::string detected = yolo.getObjectName(CameraSource::WRIST, true);

        //     if (!detected.empty()) {
        //         float confidence = yolo.getConfidence();
        //         RCLCPP_INFO(node->get_logger(), "YOLO detected: %s with confidence %.2f", detected.c_str(), confidence);
        //     } else {
        //         RCLCPP_INFO(node->get_logger(), "YOLO did not detect any objects");
        //     }
        // }
        
        // Intermediate safe position (carry position) (CHANGE THIS BECAUSE OAKD STILL DETECTS!!!)
        arm.moveToCartesianPose(0.028, -0.145, 0.139, 
                -0.289, -0.407, -0.674, 0.544);
        arm.moveToCartesianPose(0.028, -0.156, 0.243,-0.219, -0.303, -0.721, 0.584);    /***YOUR CODE HERE***/
        break;


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (outfile.is_open()) {
        outfile.close();
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
