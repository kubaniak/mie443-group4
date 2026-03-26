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
    // ArmController arm(node);
    AprilTagDetector apriltag(node);

    std::ofstream outfile("contest2_output.txt", std::ios_base::app);
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
    {
        // Spin parameters
        const double angular_speed = 0.5;          // rad/s  (positive = counter-clockwise)
        const double total_angle   = 2.0 * M_PI;   // one full rotation
        const double spin_duration = total_angle / angular_speed;  // seconds

        RCLCPP_INFO(node->get_logger(),
                     "Spinning 360 degrees in place for AMCL convergence (%.1f s at %.2f rad/s)...",
                     spin_duration, angular_speed);

        auto spin_start = node->get_clock()->now();

        while (rclcpp::ok()) {
            rclcpp::spin_some(node);  // process AMCL callbacks while spinning

            double elapsed = (node->get_clock()->now() - spin_start).seconds();
            if (elapsed >= spin_duration) {
                break;
            }

            geometry_msgs::msg::TwistStamped cmd;
            cmd.header.stamp = node->now();
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = angular_speed;
            vel_pub->publish(cmd);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Stop the robot
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = node->now();
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        vel_pub->publish(stop_cmd);

        // Let a few more AMCL updates arrive after stopping
        for (int i = 0; i < 20; ++i) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_INFO(node->get_logger(), "Spin complete. AMCL should now have a good pose estimate.");
    }

    double start_x = robotPose.x;
    double start_y = robotPose.y;
    double start_phi = robotPose.phi;
    RCLCPP_INFO(node->get_logger(), "Recorded start pose: x=%.2f, y=%.2f, phi=%.2f", start_x, start_y, start_phi);

    // State variables for contest strategy
    bool pickup_done = false;
    bool object_placed = false;
    int current_box_idx = 0;
    bool return_home = false;
    std::string manipulable_object = "cup";
    bool all_boxes_visited = false;
    bool manipulate = false;
    bool use_nav2_for_tag_align = false;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
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
        //     std::string detected = yolo.getObjectName(CameraSource::WRIST, true);

        //     if (!detected.empty()) {
        //         float confidence = yolo.getConfidence();
        //         RCLCPP_INFO(node->get_logger(), "YOLO detected: %s with confidence %.2f", detected.c_str(), confidence);
        //     } else {
        //         RCLCPP_INFO(node->get_logger(), "YOLO did not detect any objects");
        //     }
        // }

        /***YOUR CODE HERE***/
        // if (!pickup_done) {
        //     // 1. Detection and Pickup
            
        //     // STILL WORKING ON ARM POSES

        //     RCLCPP_INFO(node->get_logger(), "Moving arm to pickup position");
        //     arm.openGripper();
        //     if (arm.moveToCartesianPose(0.151, -0.013, 0.155, -0.005, 0.139, 0.037, 0.990)) {
        //         arm.closeGripper();
        //         RCLCPP_INFO(node->get_logger(), "Object picked up successfully.");
        //         RCLCPP_INFO(node->get_logger(), "Attempting to detect manipulable object with WRIST camera");
        //         // Save the image once we take the selfie
        //         manipulable_object = yolo.getObjectName(CameraSource::WRIST, true);

        //         if (manipulable_object.empty()) {
        //             RCLCPP_WARN(node->get_logger(), "Failed to detect manipulable object. NOT Retrying...");
        //             std::this_thread::sleep_for(std::chrono::seconds(1));
        //         }

        //         float confidence = yolo.getConfidence();
        //         RCLCPP_INFO(node->get_logger(), "Detected manipulable object: %s (confidence: %.2f)", manipulable_object.c_str(), confidence);

        //         // Move arm to a safe carry position (done)
        //         arm.moveToCartesianPose(0.019, -0.278, 0.243, -0.468, -0.462, -0.533, 0.533);  // home/carry position
        //     } else {
        //         RCLCPP_ERROR(node->get_logger(), "Failed to move arm to pickup position!");
        //     }

        //     pickup_done = true;
        // }
        if (!all_boxes_visited && !return_home) {
            // 2. Navigation, Identification, and Placement
            if (current_box_idx < boxes.coords.size()) {
                double box_x = boxes.coords[current_box_idx][0];
                double box_y = boxes.coords[current_box_idx][1];
                double box_phi = boxes.coords[current_box_idx][2];

                // Configure offset
                double box_offset = 0.7;

                // Adjust the target based on the offset and orientation
                double target_x = box_x + box_offset * cos(box_phi);
                double target_y = box_y + box_offset * sin(box_phi);
                double target_phi = box_phi + M_PI; // Face the box

                // Normalize target_phi to be between -PI and PI
                if (target_phi > M_PI) {
                    target_phi -= 2 * M_PI;
                } else if (target_phi < -M_PI) {
                    target_phi += 2 * M_PI;
                }

                RCLCPP_INFO(node->get_logger(), "Navigating to offset of box %d at (%.2f, %.2f, %.2f)",
                            current_box_idx, target_x, target_y, target_phi);

                if (nav.moveToGoal(target_x, target_y, target_phi)) {
                    RCLCPP_INFO(node->get_logger(), "Reached box %d", current_box_idx);

                    // Detect scene object
                    // We call getObjectName with true to save the detected image
                    std::string scene_object = yolo.getObjectName(CameraSource::OAKD, true);
                    float confidence = yolo.getConfidence();
                    const float min_detection_confidence = 0.6f;

                    if (!scene_object.empty() && confidence > min_detection_confidence) {
                        RCLCPP_INFO(node->get_logger(), "Detected scene object: %s (%.2f)", scene_object.c_str(), confidence);

                        // Log detection
                        if (outfile.is_open()) {
                            outfile << "Box " << current_box_idx << " at ("
                                    << target_x << ", " << target_y << ", " << target_phi << "): "
                                    << scene_object << " (confidence: " << confidence << ")" << std::endl;
                        }

                        if (scene_object == manipulable_object && !object_placed) {
                            manipulate = true;
                        }
                    } else {
                        RCLCPP_WARN(node->get_logger(), "No scene object detected at box %d. Retrying...", current_box_idx);
                        const double pan_speed = 0.08;  // rad/s
                        const int pan_steps = 5;
                        const int pan_step_ms = 900;
                        const int settle_ms = 250;
                        double scan_target_x = target_x;
                        double scan_target_y = target_y;
                        double scan_target_phi = target_phi;

                        auto run_panning_scan = [&](const std::string& attempt_label) -> bool {
                            auto try_detection = [&](const std::string& phase, int step_idx) -> bool {
                                RCLCPP_INFO(node->get_logger(), "%s %s step %d: running YOLO",
                                            attempt_label.c_str(), phase.c_str(), step_idx);
                                scene_object = yolo.getObjectName(CameraSource::OAKD, true);
                                confidence = yolo.getConfidence();

                                if (!scene_object.empty() && confidence > min_detection_confidence) {
                                    RCLCPP_INFO(node->get_logger(), "Detected scene object: %s (%.2f)", scene_object.c_str(), confidence);
                                    if (outfile.is_open()) {
                                        outfile << "Box " << current_box_idx << " at ("
                                                << scan_target_x << ", " << scan_target_y << ", " << scan_target_phi << "): "
                                                << scene_object << " (confidence: " << confidence << ")" << std::endl;
                                    }
                                    if (scene_object == manipulable_object && !object_placed) {
                                        manipulate = true;
                                    }
                                    return true;
                                }

                                RCLCPP_WARN(node->get_logger(), "No scene object detected at box %d during %s %s step %d.",
                                            current_box_idx, attempt_label.c_str(), phase.c_str(), step_idx);
                                if (outfile.is_open()) {
                                    outfile << "Box " << current_box_idx << " at ("
                                            << scan_target_x << ", " << scan_target_y << ", " << scan_target_phi << "): "
                                            << "None detected (" << attempt_label << " " << phase
                                            << " step " << step_idx << ")" << std::endl;
                                }
                                return false;
                            };

                            bool detected_in_scan = false;

                            RCLCPP_INFO(node->get_logger(), "Realigning to target yaw before %s scan retries...", attempt_label.c_str());
                            if (!nav.moveToGoal(scan_target_x, scan_target_y, scan_target_phi)) {
                                RCLCPP_WARN(node->get_logger(),
                                            "Failed to re-align to target yaw before %s scanning at box %d.",
                                            attempt_label.c_str(), current_box_idx);
                            }

                            // Try once from centered view before panning.
                            detected_in_scan = try_detection("center", 0);

                            for (int side = 0; side < 2 && !detected_in_scan; ++side) {
                                const bool scan_left = (side == 0);
                                const std::string phase = scan_left ? "left" : "right";
                                const double direction = scan_left ? 1.0 : -1.0;

                                for (int step = 1; step <= pan_steps && !detected_in_scan; ++step) {
                                    geometry_msgs::msg::TwistStamped cmd;
                                    cmd.header.stamp = node->now();
                                    cmd.twist.linear.x = 0.0;
                                    cmd.twist.angular.z = direction * pan_speed;
                                    vel_pub->publish(cmd);
                                    std::this_thread::sleep_for(std::chrono::milliseconds(pan_step_ms));

                                    geometry_msgs::msg::TwistStamped stop_cmd;
                                    stop_cmd.header.stamp = node->now();
                                    stop_cmd.twist.linear.x = 0.0;
                                    stop_cmd.twist.angular.z = 0.0;
                                    vel_pub->publish(stop_cmd);
                                    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));

                                    detected_in_scan = try_detection(phase, step);
                                }

                                if (!detected_in_scan) {
                                    RCLCPP_INFO(node->get_logger(), "Re-centering to target yaw after %s scan.", phase.c_str());
                                    if (!nav.moveToGoal(scan_target_x, scan_target_y, scan_target_phi)) {
                                        RCLCPP_WARN(node->get_logger(),
                                                    "Failed to re-center after %s scan at box %d.",
                                                    phase.c_str(), current_box_idx);
                                    }
                                }
                            }

                            // Ensure robot is stopped after scan motions.
                            geometry_msgs::msg::TwistStamped final_stop_cmd;
                            final_stop_cmd.header.stamp = node->now();
                            final_stop_cmd.twist.linear.x = 0.0;
                            final_stop_cmd.twist.angular.z = 0.0;
                            vel_pub->publish(final_stop_cmd);

                            return detected_in_scan;
                        };

                        bool detected_in_scan = run_panning_scan("initial");

                        // If the first scan misses, move closer and run the same panning procedure again.
                        if (!detected_in_scan) {
                            const double closer_offset = 0.3;
                            scan_target_x = box_x + closer_offset * cos(box_phi);
                            scan_target_y = box_y + closer_offset * sin(box_phi);
                            scan_target_phi = target_phi;

                            RCLCPP_INFO(node->get_logger(),
                                        "Initial panning found nothing at box %d. Moving closer and retrying scan at (%.2f, %.2f, %.2f).",
                                        current_box_idx, scan_target_x, scan_target_y, scan_target_phi);

                            if (nav.moveToGoal(scan_target_x, scan_target_y, scan_target_phi)) {
                                detected_in_scan = run_panning_scan("closer");
                            } else {
                                RCLCPP_WARN(node->get_logger(), "Failed to move closer for retry scan at box %d.", current_box_idx);
                            }
                        }
                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Failed to navigate to box %d", current_box_idx);
                }
                if (manipulate) {
                    RCLCPP_INFO(node->get_logger(), "Cup detected, Attempting to place the manipulable object in the bin at box %d", current_box_idx);

                    // Look for AprilTags for the bin
                    std::vector<int> candidate_tags = {0, 1, 2, 3, 4};
                    std::vector<int> visible_tags = apriltag.getVisibleTags(candidate_tags, 100);

                    if (!visible_tags.empty()) {
                        RCLCPP_INFO(node->get_logger(), "Found bin with AprilTag ID: %d. Aligning with tag...", visible_tags[0]);

                        auto tag_pose_opt = apriltag.getTagPose(visible_tags[0]);
                        if (tag_pose_opt.has_value()) {
                            auto tag_pose = tag_pose_opt.value();
                            double tag_x = tag_pose.position.x;
                            double tag_y = tag_pose.position.y;
                            RCLCPP_INFO(node->get_logger(), "Tag relative pose: x=%.2f, y=%.2f", tag_x, tag_y);

                            // Placeholder values for desired distance and offset from tag
                            double desired_distance_x = 0.3; // Stop 0.3m in front of the tag
                            double desired_distance_y = 0.0; // Centered with the tag

                            double error_x = tag_x - desired_distance_x;
                            double error_y = tag_y - desired_distance_y;

                            if (use_nav2_for_tag_align) {
                                // Variation 1: Use Nav2
                                RCLCPP_INFO(node->get_logger(), "Using Nav2 to align with tag...");

                                // Calculate global pose
                                // tag_x and tag_y are in base_link.
                                // To move base_link by (error_x, error_y):
                                double goal_x = robotPose.x + error_x * cos(robotPose.phi) - error_y * sin(robotPose.phi);
                                double goal_y = robotPose.y + error_x * sin(robotPose.phi) + error_y * cos(robotPose.phi);
                                double goal_phi = robotPose.phi; // Keep current orientation or face the tag

                                RCLCPP_INFO(node->get_logger(), "Aligning to global pose: x=%.2f, y=%.2f", goal_x, goal_y);
                                if (!nav.moveToGoal(goal_x, goal_y, goal_phi)) {
                                    RCLCPP_WARN(node->get_logger(), "Failed to align using Nav2!");
                                }
                            } else {
                                // Variation 2: Use Control Loop
                                RCLCPP_INFO(node->get_logger(), "Using Control Loop to align with tag...");

                                const double kP_linear = 0.5;
                                const double kP_angular = 1.0;
                                const double dist_tolerance = 0.05;
                                const double angle_tolerance = 0.05;

                                rclcpp::Rate loop_rate(10); // 10 Hz
                                auto align_start = std::chrono::system_clock::now();

                                while (rclcpp::ok()) {
                                    auto align_now = std::chrono::system_clock::now();
                                    if (std::chrono::duration_cast<std::chrono::seconds>(align_now - align_start).count() > 10) {
                                        RCLCPP_WARN(node->get_logger(), "Alignment timeout!");
                                        break;
                                    }

                                    // Refresh tag pose
                                    auto current_tag_pose_opt = apriltag.getTagPose(visible_tags[0]);
                                    if (!current_tag_pose_opt.has_value()) {
                                        RCLCPP_WARN(node->get_logger(), "Lost sight of tag during alignment!");
                                        break;
                                    }

                                    auto current_tag_pose = current_tag_pose_opt.value();
                                    double curr_tag_x = current_tag_pose.position.x;
                                    double curr_tag_y = current_tag_pose.position.y;

                                    double err_x = curr_tag_x - desired_distance_x;
                                    // In addition to lateral error (Y), let's calculate the heading error to ensure we are facing the tag squarely.
                                    // The tag's orientation in base_link tells us how much we need to rotate to face it.

                                    // Get yaw from quaternion
                                    tf2::Quaternion q(
                                        current_tag_pose.orientation.x,
                                        current_tag_pose.orientation.y,
                                        current_tag_pose.orientation.z,
                                        current_tag_pose.orientation.w);
                                    tf2::Matrix3x3 m(q);
                                    double roll, pitch, yaw;
                                    m.getRPY(roll, pitch, yaw);

                                    // yaw is the angle of the tag's Z-axis (forward) relative to our X-axis.
                                    // If we want to face the tag directly, our desired relative yaw should be pi or -pi (depending on tag orientation convention)
                                    // Wait, the tag's Z axis normally points OUT of the tag. If we want to face it, the tag's Z should point towards us (180 degrees from our X).
                                    // Or simply use the tag's relative X and Y to compute the angle we need to turn: atan2(Y, X).
                                    // The lateral error (Y) gets smaller as we point towards it, but we could be pointing towards it while being offset sideways.
                                    // Using atan2(Y, X) is a simple heading controller. Let's use that as the primary angular error.

                                    double heading_error = std::atan2(curr_tag_y, curr_tag_x);
                                    double err_y = curr_tag_y - desired_distance_y;

                                    if (std::abs(err_x) < dist_tolerance && std::abs(heading_error) < angle_tolerance) {
                                        RCLCPP_INFO(node->get_logger(), "Successfully aligned with tag.");
                                        break;
                                    }

                                    geometry_msgs::msg::TwistStamped cmd;
                                    cmd.header.stamp = node->now();
                                    // Move forward/backward based on error in X
                                    cmd.twist.linear.x = kP_linear * err_x;
                                    // Rotate to correct heading error (pointing towards the tag)
                                    cmd.twist.angular.z = kP_angular * heading_error;

                                    // Cap speeds
                                    if (cmd.twist.linear.x > 0.2) cmd.twist.linear.x = 0.2;
                                    if (cmd.twist.linear.x < -0.2) cmd.twist.linear.x = -0.2;
                                    if (cmd.twist.angular.z > 0.5) cmd.twist.angular.z = 0.5;
                                    if (cmd.twist.angular.z < -0.5) cmd.twist.angular.z = -0.5;

                                    vel_pub->publish(cmd);

                                    rclcpp::spin_some(node);
                                    loop_rate.sleep();
                                }

                                // Stop robot
                                geometry_msgs::msg::TwistStamped stop_cmd;
                                stop_cmd.header.stamp = node->now();
                                stop_cmd.twist.linear.x = 0.0;
                                stop_cmd.twist.angular.z = 0.0;
                                vel_pub->publish(stop_cmd);
                            }
                        } else {
                            RCLCPP_WARN(node->get_logger(), "Failed to get pose for visible tag %d!", visible_tags[0]);
                        }
                    }
                    else {
                        RCLCPP_WARN(node->get_logger(), "Cup detected but no AprilTag found for the bin at box %d!", current_box_idx);
                    }
                    // // Check if it matches the manipulable object, and only place if not placed yet
                        // if (scene_object == manipulable_object && !object_placed) {
                        //     RCLCPP_INFO(node->get_logger(), "MATCH FOUND! Manipulable object matches scene object.");

                        //     // Look for AprilTags for the bin
                        //     std::vector<int> candidate_tags = {0, 1, 2, 3, 4};
                        //     std::vector<int> visible_tags = apriltag.getVisibleTags(candidate_tags, 100);

                        //     // TODO: Still not doing anything with april tags. Could use tf to determine exact bin location and update drop pose accordingly, but for now just check if any tag is visible at all as a proxy for "is the bin in front of us"
                        //     if (!visible_tags.empty()) {
                        //         RCLCPP_INFO(node->get_logger(), "Found bin with AprilTag ID: %d. Placing object.", visible_tags[0]);

                        //         // TODO: Update values for drop_pose (x, y, z, qx, qy, qz, qw) (could just be home/carry position)
                        //         double drop_x = 0.4;
                        //         double drop_y = 0.0;
                        //         double drop_z = 0.2;
                        //         // Use a valid, normalized quaternion (identity orientation)
                        //         double drop_qx = 0.0;
                        //         double drop_qy = 0.0;
                        //         double drop_qz = 0.0;
                        //         double drop_qw = 1.0;

                        //         if (arm.moveToCartesianPose(drop_x, drop_y, drop_z, drop_qx, drop_qy, drop_qz, drop_qw)) {
                        //             arm.openGripper();
                        //             RCLCPP_INFO(node->get_logger(), "Object placed successfully in the bin!");

                        //             // Move arm back to carry/rest position
                        //             arm.moveToCartesianPose(0.019, -0.278, 0.243, -0.468, -0.462, -0.533, 0.533);  // home/carry position

                        //             object_placed = true;
                        //         } else {
                        //             RCLCPP_ERROR(node->get_logger(), "Failed to move arm to drop position!");
                        //         }
                        //     } else {
                        //         RCLCPP_WARN(node->get_logger(), "Matched object, but no AprilTag found for the bin!");
                        //     }
                        // } else if (scene_object == manipulable_object && object_placed) {
                        //     RCLCPP_INFO(node->get_logger(), "Matched object found again, but already placed the manipulable object.");
                        // }
                }
                current_box_idx++;
            } else {
                all_boxes_visited = true;
                if (!object_placed) {
                    RCLCPP_WARN(node->get_logger(),
                                "All boxes have been visited without successfully placing the object. Returning home.");
                    if (outfile.is_open()) {
                        outfile << "WARNING: All boxes visited; manipulable object was not placed. Returning home."
                                << std::endl;
                    }
                } else {
                    RCLCPP_INFO(node->get_logger(), "All boxes visited and manipulable object placed! Returning home.");
                }
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
