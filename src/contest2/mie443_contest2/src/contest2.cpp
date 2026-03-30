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
#include <algorithm>
#include <limits>
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
    Navigation nav(node);
    YoloInterface yolo(node);
    ArmController arm(node);
    AprilTagDetector apriltag(node);

    std::ofstream outfile("src/contest2/mie443_contest2/contest2_output.txt", std::ios_base::app);
    if (!outfile.is_open()) {
        RCLCPP_WARN(node->get_logger(), "Could not open contest2_output.txt for writing");
    }

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    const uint64_t contest_duration_sec = 300;
    const uint64_t return_home_buffer_sec = 20;



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
    const int max_nav_attempts_per_box = 3;
    const int max_detection_attempts_per_box = 2;
    const int max_apriltag_alignment_failures_per_box = 2;
    std::vector<int> nav_attempts_per_box(boxes.coords.size(), 0);
    std::vector<int> detection_attempts_per_box(boxes.coords.size(), 0);
    std::vector<int> apriltag_alignment_failures_per_box(boxes.coords.size(), 0);
    std::vector<int> deferred_boxes;
    std::vector<bool> box_is_deferred(boxes.coords.size(), false);
    std::vector<bool> box_is_finalized(boxes.coords.size(), false);

    // Desired camera->tag alignment setpoints.
    const double target_tag_x = node->declare_parameter<double>("target_x", -0.009);
    const double target_tag_z = node->declare_parameter<double>("target_z", 0.196);

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

    auto format_deferred_boxes = [&]() -> std::string {
        if (deferred_boxes.empty()) {
            return "none";
        }
        std::ostringstream oss;
        for (size_t i = 0; i < deferred_boxes.size(); ++i) {
            if (i > 0) {
                oss << ",";
            }
            oss << deferred_boxes[i];
        }
        return oss.str();
    };

    auto process_scene_detection = [&](int box_idx,
                                       const std::string& scene_object,
                                       float confidence,
                                       const std::string& context_label,
                                       double log_x,
                                       double log_y,
                                       double log_phi,
                                       float min_detection_confidence) -> bool {
        if (scene_object.empty() || confidence <= min_detection_confidence) {
            return false;
        }

        const bool already_checked_off =
            (remaining_scene_objects.find(scene_object) == remaining_scene_objects.end());
        const bool needs_manipulable_retry =
            (scene_object == manipulable_object && !object_placed);

        // Keep duplicate suppression for scene scoring, but still allow cup
        // detections to trigger placement retries until the cup is placed.
        if (already_checked_off && !needs_manipulable_retry) {
            RCLCPP_INFO(node->get_logger(),
                        "Ignoring already checked-off object '%s' (%.2f) during %s.",
                        scene_object.c_str(), confidence, context_label.c_str());
            // if (outfile.is_open()) {
            //     outfile << "Box " << box_idx << " at ("
            //             << log_x << ", " << log_y << ", " << log_phi << "): "
            //             << "IGNORED checked-off object " << scene_object
            //             << " (confidence: " << confidence << ", " << context_label << ")" << std::endl;
            // }
            return false;
        }

        if (!already_checked_off) {
            remaining_scene_objects.erase(scene_object);
            RCLCPP_INFO(node->get_logger(),
                        "Detected new scene object: %s (%.2f) during %s. Remaining: [%s]",
                        scene_object.c_str(), confidence, context_label.c_str(),
                        format_remaining_scene_objects().c_str());

            if (outfile.is_open()) {
                outfile << "Box " << box_idx << " at ("
                        << log_x << ", " << log_y << ", " << log_phi << "): "
                        << scene_object << " (confidence: " << confidence << ", " << context_label << ")"
                        << std::endl;
                // outfile << "CHECKED_OFF: " << scene_object
                //         << " | Remaining: [" << format_remaining_scene_objects() << "]" << std::endl;
            }
        } else if (needs_manipulable_retry) {
            RCLCPP_INFO(node->get_logger(),
                        "Re-detected manipulable object '%s' (%.2f) during %s for placement retry.",
                        scene_object.c_str(), confidence, context_label.c_str());
        }

        if (scene_object == manipulable_object && !object_placed) {
            RCLCPP_INFO(node->get_logger(), "Manipulable object '%s' detected! Will attempt to place in bin at box %d.",
                        manipulable_object.c_str(), box_idx);
        }
        return true;
    };

    auto get_seconds_remaining = [&]() -> uint64_t {
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        return (elapsed >= contest_duration_sec) ? 0 : (contest_duration_sec - elapsed);
    };

    // Try to align with the bin AprilTag from the OAK-D camera frame.
    auto try_align_to_bin_apriltag = [&](int box_idx, int& aligned_tag_id) -> bool {
        const std::vector<int> candidate_tags = {0, 1, 2, 3, 4};
        const int max_alignment_iters = 40;
        const int control_period_ms = 100;
        const int stable_cycles_required = 3;

        const double k_angular = 1.8;
        const double k_linear = 0.8;
        const double max_angular = 0.7;
        const double max_linear = 0.2;
        const double x_tolerance = 0.015;
        const double z_tolerance = 0.03;

        bool observed_valid_transform = false;
        int stable_cycles = 0;
        aligned_tag_id = -1;

        apriltag.setReferenceFrame("oakd_rgb_camera_optical_frame");

        for (int iter = 0; iter < max_alignment_iters && rclcpp::ok(); ++iter) {
            std::vector<int> visible_tags = apriltag.getVisibleTags(candidate_tags, 50);
            if (!visible_tags.empty()) {
                aligned_tag_id = visible_tags.front();
            }

            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = node->now();
            cmd_vel.header.frame_id = "base_link";
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;

            if (aligned_tag_id < 0) {
                // Slowly rotate in place until at least one candidate tag becomes visible.
                cmd_vel.twist.angular.z = 0.2;
                vel_pub->publish(cmd_vel);
                stable_cycles = 0;
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(control_period_ms));
                continue;
            }

            auto tag_tf_opt = apriltag.getTagTransform(aligned_tag_id, 50);
            if (!tag_tf_opt.has_value()) {
                cmd_vel.twist.angular.z = 0.15;
                vel_pub->publish(cmd_vel);
                stable_cycles = 0;
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(control_period_ms));
                continue;
            }

            observed_valid_transform = true;

            const auto & t = tag_tf_opt->transform.translation;
            const double x_error = t.x - target_tag_x;
            const double z_error = t.z - target_tag_z;

            // In camera optical frame, +x means tag appears to the right.
            cmd_vel.twist.angular.z = std::clamp(-k_angular * x_error, -max_angular, max_angular);
            cmd_vel.twist.linear.x = std::clamp(k_linear * z_error, -max_linear, max_linear);
            vel_pub->publish(cmd_vel);

            if (std::abs(x_error) < x_tolerance && std::abs(z_error) < z_tolerance) {
                stable_cycles += 1;
                if (stable_cycles >= stable_cycles_required) {
                    geometry_msgs::msg::TwistStamped stop_cmd;
                    stop_cmd.header.stamp = node->now();
                    stop_cmd.header.frame_id = "base_link";
                    vel_pub->publish(stop_cmd);

                    RCLCPP_INFO(node->get_logger(),
                                "Aligned to AprilTag %d at box %d (x_error=%.3f, z_error=%.3f)",
                                aligned_tag_id, box_idx, x_error, z_error);
                    return true;
                }
            } else {
                stable_cycles = 0;
            }

            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(control_period_ms));
        }

        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = node->now();
        stop_cmd.header.frame_id = "base_link";
        vel_pub->publish(stop_cmd);

        if (!observed_valid_transform) {
            RCLCPP_WARN(node->get_logger(),
                        "AprilTag alignment failed at box %d: no valid tag transform observed before timeout.",
                        box_idx);
        } else {
            RCLCPP_WARN(node->get_logger(),
                        "AprilTag alignment failed at box %d: transform seen but convergence tolerance not met before timeout.",
                        box_idx);
        }
        return false;
    };

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= contest_duration_sec) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        const uint64_t seconds_remaining =
            (secondsElapsed >= contest_duration_sec) ? 0 : (contest_duration_sec - secondsElapsed);
        static uint64_t last_status_log_second = std::numeric_limits<uint64_t>::max();

        if (secondsElapsed != last_status_log_second) {
            last_status_log_second = secondsElapsed;
            RCLCPP_INFO(node->get_logger(),
                        "Status: t=%lus remaining=%lus primary_idx=%d deferred=[%s] return_home=%s placed=%s",
                        secondsElapsed,
                        seconds_remaining,
                        current_box_idx,
                        format_deferred_boxes().c_str(),
                        return_home ? "true" : "false",
                        object_placed ? "true" : "false");
        }

        if (!return_home && seconds_remaining <= return_home_buffer_sec) {
            RCLCPP_WARN(node->get_logger(),
                        "%lu seconds remaining. Stopping box search and returning home now.",
                        seconds_remaining);
            return_home = true;
            all_boxes_visited = true;
        }
        
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
        if (!pickup_done && !return_home) {
            // 1. Detection and Pickup
            
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

            const int total_yolo_attempts = 5;
            for (int attempt = 1; attempt <= total_yolo_attempts; ++attempt) {
                if (attempt > 1) {
                    RCLCPP_INFO(node->get_logger(), "Waiting 2 seconds for the next YOLO frame...");
                    
                    // --- NON-BLOCKING WAIT LOGIC ---
                    auto wait_start = node->get_clock()->now();
                    // Loop for 2 seconds while actively processing ROS callbacks
                    while (rclcpp::ok() && (node->get_clock()->now() - wait_start).seconds() < 2.0) {
                        rclcpp::spin_some(node); // Process incoming camera/YOLO messages!
                        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Prevent CPU pegging
                    }
                }
                RCLCPP_INFO(node->get_logger(),
                            "Attempting YOLO (OAKD Camera) detection %d/%d at %lu seconds",
                            attempt,
                            total_yolo_attempts,
                            secondsElapsed);
                
                std::string detected = yolo.getObjectName(CameraSource::OAKD, true);

                if (!detected.empty()) {
                    float confidence = yolo.getConfidence();
                    RCLCPP_INFO(node->get_logger(), "YOLO detected: %s with confidence %.2f", detected.c_str(), confidence);
                } else {
                    RCLCPP_INFO(node->get_logger(), "YOLO did not detect any objects");
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Intermediate safe position (carry position)
            arm.moveToCartesianPose(0.028, -0.156, 0.243,
                -0.219, -0.303, -0.721, 0.584);

            pickup_done = true;

        } else if (!all_boxes_visited && !return_home) {
            // 2. Navigation, Identification, and Placement
            int active_box_idx = -1;
            bool from_deferred_queue = false;

            auto move_with_time_budget = [&](double goal_x,
                                             double goal_y,
                                             double goal_phi,
                                             const std::string& context_label) -> bool {
                const uint64_t remaining = get_seconds_remaining();
                if (remaining <= return_home_buffer_sec + 2) {
                    RCLCPP_WARN(node->get_logger(),
                                "Skipping %s: only %lu seconds remain; forcing return-home.",
                                context_label.c_str(),
                                remaining);
                    return_home = true;
                    all_boxes_visited = true;
                    return false;
                }

                const uint64_t available_for_this_goal = remaining - return_home_buffer_sec - 1;
                const uint64_t timeout_sec = std::max<uint64_t>(1, std::min<uint64_t>(available_for_this_goal, 45));
                const int timeout_ms = static_cast<int>(timeout_sec * 1000);

                return nav.moveToGoal(goal_x, goal_y, goal_phi, timeout_ms);
            };

            if (current_box_idx < static_cast<int>(boxes.coords.size())) {
                active_box_idx = current_box_idx;
            } else if (!deferred_boxes.empty()) {
                active_box_idx = deferred_boxes.front();
                deferred_boxes.erase(deferred_boxes.begin());
                box_is_deferred[active_box_idx] = false;
                from_deferred_queue = true;
                RCLCPP_INFO(node->get_logger(), "Revisiting deferred box %d", active_box_idx);
            }

            if (active_box_idx >= 0) {
                double box_x = boxes.coords[active_box_idx][0];
                double box_y = boxes.coords[active_box_idx][1];
                double box_phi = boxes.coords[active_box_idx][2];

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
                            active_box_idx, target_x, target_y, target_phi);

                // Keep latest detection available for optional manipulation logic below.
                std::string scene_object;
                float confidence = 0.0f;
                const float min_detection_confidence = 0.6f;
                bool detected_scene_object = false;
                bool finalize_box = false;
                bool defer_box = false;

                auto schedule_deferred_visit = [&](int box_idx, const std::string& reason) {
                    if (!box_is_deferred[box_idx] && !box_is_finalized[box_idx]) {
                        deferred_boxes.push_back(box_idx);
                        box_is_deferred[box_idx] = true;
                        RCLCPP_WARN(node->get_logger(),
                                    "Deferring box %d for revisit (%s). Deferred queue size: %zu",
                                    box_idx,
                                    reason.c_str(),
                                    deferred_boxes.size());
                    }
                };

                if (move_with_time_budget(target_x, target_y, target_phi, "box approach navigation")) {
                    RCLCPP_INFO(node->get_logger(), "Reached box %d", active_box_idx);
                    nav_attempts_per_box[active_box_idx] = 0;

                    // Detect scene object
                    // We call getObjectName with true to save the detected image
                    scene_object = yolo.getObjectName(CameraSource::OAKD, true);
                    confidence = yolo.getConfidence();

                    detected_scene_object = process_scene_detection(
                        active_box_idx,
                        scene_object,
                        confidence,
                        "initial view",
                        target_x,
                        target_y,
                        target_phi,
                        min_detection_confidence);

                    if (!detected_scene_object) {
                        RCLCPP_WARN(node->get_logger(),
                                    "No usable scene object detected at box %d. Running local scan.",
                                    active_box_idx);
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

                                const std::string attempt_context =
                                    attempt_label + " " + phase + " step " + std::to_string(step_idx);
                                if (process_scene_detection(
                                        active_box_idx,
                                        scene_object,
                                        confidence,
                                        attempt_context,
                                        scan_target_x,
                                        scan_target_y,
                                        scan_target_phi,
                                        min_detection_confidence)) {
                                    return true;
                                }

                                RCLCPP_WARN(node->get_logger(), "No usable scene object detected at box %d during %s %s step %d.",
                                            active_box_idx, attempt_label.c_str(), phase.c_str(), step_idx);
                                return false;
                            };

                            bool detected_in_scan = false;

                            RCLCPP_INFO(node->get_logger(), "Realigning to target yaw before %s scan retries...", attempt_label.c_str());
                            if (!move_with_time_budget(
                                    scan_target_x,
                                    scan_target_y,
                                    scan_target_phi,
                                    attempt_label + " yaw realignment")) {
                                if (return_home) {
                                    return false;
                                }
                                RCLCPP_WARN(node->get_logger(),
                                            "Failed to re-align to target yaw before %s scanning at box %d.",
                                            attempt_label.c_str(), active_box_idx);
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
                                    if (!move_with_time_budget(
                                            scan_target_x,
                                            scan_target_y,
                                            scan_target_phi,
                                            phase + " scan recenter")) {
                                        if (return_home) {
                                            return false;
                                        }
                                        RCLCPP_WARN(node->get_logger(),
                                                    "Failed to re-center after %s scan at box %d.",
                                                    phase.c_str(), active_box_idx);
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
                                        active_box_idx, scan_target_x, scan_target_y, scan_target_phi);

                            if (move_with_time_budget(
                                    scan_target_x,
                                    scan_target_y,
                                    scan_target_phi,
                                    "closer-range scan approach")) {
                                detected_in_scan = run_panning_scan("closer");
                            } else if (return_home) {
                                detected_in_scan = false;
                            } else {
                                RCLCPP_WARN(node->get_logger(), "Failed to move closer for retry scan at box %d.", active_box_idx);
                            }
                        }

                        detected_scene_object = detected_in_scan;
                    }

                    if (detected_scene_object) {
                        detection_attempts_per_box[active_box_idx] = 0;
                        finalize_box = true;
                    } else {
                        detection_attempts_per_box[active_box_idx] += 1;
                        if (detection_attempts_per_box[active_box_idx] >= max_detection_attempts_per_box) {
                            RCLCPP_ERROR(node->get_logger(),
                                         "Box %d had no usable detection after %d full attempts. Marking unresolved and moving on.",
                                         active_box_idx,
                                         detection_attempts_per_box[active_box_idx]);
                            finalize_box = true;
                        } else {
                            defer_box = true;
                        }
                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Failed to navigate to box %d", active_box_idx);
                    nav_attempts_per_box[active_box_idx] += 1;
                    if (nav_attempts_per_box[active_box_idx] >= max_nav_attempts_per_box) {
                        RCLCPP_ERROR(node->get_logger(),
                                     "Box %d navigation failed %d times. Marking unreachable and moving on.",
                                     active_box_idx,
                                     nav_attempts_per_box[active_box_idx]);
                        finalize_box = true;
                    } else {
                        defer_box = true;
                    }
                }

                if (detected_scene_object && scene_object == manipulable_object && !object_placed) {
                    RCLCPP_INFO(node->get_logger(),
                                "Cup detected at box %d. Attempting AprilTag alignment for bin placement.",
                                active_box_idx);

                    int aligned_tag_id = -1;
                    bool aligned_to_tag = try_align_to_bin_apriltag(active_box_idx, aligned_tag_id);

                    if (aligned_to_tag) {
                        apriltag_alignment_failures_per_box[active_box_idx] = 0;

                        RCLCPP_INFO(node->get_logger(),
                                    "Tag-guided placement at box %d using tag %d.",
                                    active_box_idx,
                                    aligned_tag_id);

                        // Home Position (also drop position!)
                        arm.moveToCartesianPose(0.023, -0.277, 0.246,
                            -0.464, -0.474, -0.529, 0.529);

                        arm.openGripper();
                        object_placed = true;

                        // Intermediate safe position (carry position)
                        arm.moveToCartesianPose(0.028, -0.156, 0.243,
                            -0.219, -0.303, -0.721, 0.584);

                        finalize_box = true;
                    } else {
                        apriltag_alignment_failures_per_box[active_box_idx] += 1;
                        const int alignment_failures = apriltag_alignment_failures_per_box[active_box_idx];

                        if (alignment_failures >= max_apriltag_alignment_failures_per_box) {
                            RCLCPP_WARN(node->get_logger(),
                                        "AprilTag alignment failed %d times at cup box %d. Forcing Nav2-only drop.",
                                        alignment_failures,
                                        active_box_idx);

                            geometry_msgs::msg::TwistStamped stop_cmd;
                            stop_cmd.header.stamp = node->now();
                            stop_cmd.header.frame_id = "base_link";
                            vel_pub->publish(stop_cmd);

                            // Forced drop fallback after repeated AprilTag failures.
                            arm.moveToCartesianPose(0.023, -0.277, 0.246,
                                -0.464, -0.474, -0.529, 0.529);

                            arm.openGripper();
                            object_placed = true;

                            arm.moveToCartesianPose(0.028, -0.156, 0.243,
                                -0.219, -0.303, -0.721, 0.584);

                            finalize_box = true;
                            defer_box = false;
                        } else {
                            RCLCPP_WARN(node->get_logger(),
                                        "AprilTag alignment failed at cup box %d (attempt %d/%d). Deferring one retry.",
                                        active_box_idx,
                                        alignment_failures,
                                        max_apriltag_alignment_failures_per_box);
                            finalize_box = false;
                            defer_box = true;
                        }
                    }
                }

                if (finalize_box) {
                    box_is_finalized[active_box_idx] = true;
                    box_is_deferred[active_box_idx] = false;
                } else if (defer_box) {
                    schedule_deferred_visit(active_box_idx, "temporary failure");
                }

                if (!from_deferred_queue) {
                    current_box_idx++;
                }
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
            const uint64_t remaining_for_home = std::max<uint64_t>(get_seconds_remaining(), 10);
            const uint64_t return_timeout_sec = std::min<uint64_t>(remaining_for_home, 90);
            if (nav.moveToGoal(start_x, start_y, start_phi, static_cast<int>(return_timeout_sec * 1000))) {
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

    if (secondsElapsed > contest_duration_sec) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
