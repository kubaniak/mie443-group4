#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
    

    // Initialize helper objects
    Navigation nav(node);
    YoloInterface yolo(node);
    ArmController arm(node);
    AprilTagDetector apriltag(node);

    std::ofstream outfile("contest2_output.txt");
    if (!outfile.is_open()) {
        RCLCPP_WARN(node->get_logger(), "Could not open contest2_output.txt for writing");
    }

    // Spin for a moment to receive initial AMCL pose
    RCLCPP_INFO(node->get_logger(), "Waiting for initial AMCL pose...");
    for (int i = 0; i < 50; ++i) { // Wait up to 5 seconds
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Break early if we receive a non-zero pose (assuming start is not exactly 0,0,0)
        if (robotPose.x != 0.0 || robotPose.y != 0.0 || robotPose.phi != 0.0) {
            break;
        }
    }

    double start_x = robotPose.x;
    double start_y = robotPose.y;
    double start_phi = robotPose.phi;

    if (start_x == 0.0 && start_y == 0.0 && start_phi == 0.0) {
        RCLCPP_WARN(node->get_logger(), "AMCL pose might not be updated. Using default start pose (0, 0, 0).");
    } else {
        RCLCPP_INFO(node->get_logger(), "Recorded start pose: x=%.2f, y=%.2f, phi=%.2f", start_x, start_y, start_phi);
    }

    // State variables for contest strategy
    bool pickup_done = false;
    int current_box_idx = 0;
    bool return_home = false;
    std::string manipulable_object = "";
    bool all_boxes_visited = false;

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/
        if (!pickup_done) {
            // 1. Detection and Pickup
            RCLCPP_INFO(node->get_logger(), "Attempting to detect manipulable object with WRIST camera");
            manipulable_object = yolo.getObjectName(CameraSource::WRIST, true);

            if (manipulable_object.empty()) {
                RCLCPP_WARN(node->get_logger(), "Failed to detect manipulable object. Retrying...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            RCLCPP_INFO(node->get_logger(), "Detected manipulable object: %s", manipulable_object.c_str());

            // TODO: Use correct coordinates provided by Instructor/TAs on contest day for the arm base frame
            double pickup_x = 0.3;
            double pickup_y = 0.0;
            double pickup_z = 0.1;
            double pickup_roll = 0.0;
            double pickup_pitch = 1.57; // pointing down
            double pickup_yaw = 0.0;

            RCLCPP_INFO(node->get_logger(), "Moving arm to pickup position");
            arm.openGripper();
            if (arm.moveToCartesianPose(pickup_x, pickup_y, pickup_z, pickup_roll, pickup_pitch, pickup_yaw)) {
                arm.closeGripper();
                RCLCPP_INFO(node->get_logger(), "Object picked up successfully.");

                // Move arm to a safe carry position
                arm.moveToCartesianPose(0.1, 0.0, 0.3, 0.0, 0.0, 0.0);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to move arm to pickup position!");
            }

            pickup_done = true;
        }
        else if (!all_boxes_visited && !return_home) {
            // 2. Navigation, Identification, and Placement
            if (current_box_idx < boxes.coords.size()) {
                double target_x = boxes.coords[current_box_idx][0];
                double target_y = boxes.coords[current_box_idx][1];
                double target_phi = boxes.coords[current_box_idx][2];

                RCLCPP_INFO(node->get_logger(), "Navigating to box %d at (%.2f, %.2f, %.2f)",
                            current_box_idx, target_x, target_y, target_phi);

                if (nav.moveToGoal(target_x, target_y, target_phi)) {
                    RCLCPP_INFO(node->get_logger(), "Reached box %d", current_box_idx);

                    // Detect scene object
                    std::string scene_object = yolo.getObjectName(CameraSource::OAKD, false);
                    if (scene_object.empty()) {
                        RCLCPP_INFO(node->get_logger(), "OAKD failed to detect object. Trying WRIST camera...");
                        scene_object = yolo.getObjectName(CameraSource::WRIST, false);
                    }

                    if (!scene_object.empty()) {
                        float confidence = yolo.getConfidence();
                        RCLCPP_INFO(node->get_logger(), "Detected scene object: %s (%.2f)", scene_object.c_str(), confidence);

                        // Log detection
                        if (outfile.is_open()) {
                            outfile << "Box " << current_box_idx << " at ("
                                    << target_x << ", " << target_y << ", " << target_phi << "): "
                                    << scene_object << " (confidence: " << confidence << ")" << std::endl;
                        }

                        // Check if it matches the manipulable object
                        if (scene_object == manipulable_object) {
                            RCLCPP_INFO(node->get_logger(), "MATCH FOUND! Manipulable object matches scene object.");

                            // Look for AprilTags for the bin
                            // TODO: Pass actual candidate IDs once known
                            std::vector<int> candidate_tags = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
                            std::vector<int> visible_tags = apriltag.getVisibleTags(candidate_tags, 100);

                            if (!visible_tags.empty()) {
                                RCLCPP_INFO(node->get_logger(), "Found bin with AprilTag ID: %d. Placing object.", visible_tags[0]);

                                // TODO: Use correct coordinates to place the object in the bin
                                double drop_x = 0.4;
                                double drop_y = 0.0;
                                double drop_z = 0.2;
                                double drop_roll = 0.0;
                                double drop_pitch = 1.57; // pointing down
                                double drop_yaw = 0.0;

                                if (arm.moveToCartesianPose(drop_x, drop_y, drop_z, drop_roll, drop_pitch, drop_yaw)) {
                                    arm.openGripper();
                                    RCLCPP_INFO(node->get_logger(), "Object placed successfully in the bin!");

                                    // Move arm back to carry/rest position
                                    arm.moveToCartesianPose(0.1, 0.0, 0.3, 0.0, 0.0, 0.0);

                                    // Set state to return home
                                    return_home = true;
                                } else {
                                    RCLCPP_ERROR(node->get_logger(), "Failed to move arm to drop position!");
                                }
                            } else {
                                RCLCPP_WARN(node->get_logger(), "Matched object, but no AprilTag found for the bin!");
                            }
                        }
                    } else {
                        RCLCPP_WARN(node->get_logger(), "No scene object detected at box %d.", current_box_idx);
                        if (outfile.is_open()) {
                            outfile << "Box " << current_box_idx << " at ("
                                    << target_x << ", " << target_y << ", " << target_phi << "): "
                                    << "None detected" << std::endl;
                        }
                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Failed to navigate to box %d", current_box_idx);
                }

                current_box_idx++;
            } else {
                all_boxes_visited = true;
                return_home = true;
            }
        }
        else if (return_home) {
            // 3. Return to Start
            RCLCPP_INFO(node->get_logger(), "Returning to starting location at (%.2f, %.2f, %.2f)", start_x, start_y, start_phi);
            if (nav.moveToGoal(start_x, start_y, start_phi)) {
                RCLCPP_INFO(node->get_logger(), "Successfully returned to the starting location. Mission complete!");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to return to the starting location!");
            }
            break; // Exit the loop
        }

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
