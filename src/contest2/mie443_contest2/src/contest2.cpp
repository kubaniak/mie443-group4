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

namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

double wrapToPi(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double computeOrientationErrorAboutCameraY(
    const geometry_msgs::msg::Quaternion &q_meas,
    const geometry_msgs::msg::Quaternion &q_des) {
    tf2::Quaternion q_m;
    tf2::Quaternion q_d;
    tf2::fromMsg(q_meas, q_m);
    tf2::fromMsg(q_des, q_d);

    q_m.normalize();
    q_d.normalize();

    const tf2::Quaternion q_err = q_d.inverse() * q_m;

    // For optical frame alignment, this term is a practical proxy for in-plane skew.
    return wrapToPi(2.0 * std::atan2(q_err.y(), q_err.w()));
}

enum class AlignState {
    SEARCH,
    CENTER,
    APPROACH,
    FINAL_ALIGN,
    HOLD
};

const char *stateToString(AlignState state) {
    switch (state) {
        case AlignState::SEARCH:
            return "SEARCH";
        case AlignState::CENTER:
            return "CENTER";
        case AlignState::APPROACH:
            return "APPROACH";
        case AlignState::FINAL_ALIGN:
            return "FINAL_ALIGN";
        case AlignState::HOLD:
            return "HOLD";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

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

    // Control parameters.
    const double loop_hz = node->declare_parameter<double>("loop_hz", 20.0);
    const double max_linear = node->declare_parameter<double>("max_linear", 0.20);
    const double max_angular = node->declare_parameter<double>("max_angular", 1.00);
    const double search_angular = node->declare_parameter<double>("search_angular", 0.35);

    const double kp_z = node->declare_parameter<double>("kp_z", 1.4);
    const double kp_bearing = node->declare_parameter<double>("kp_bearing", 1.8);
    const double kp_bearing_approach = node->declare_parameter<double>("kp_bearing_approach", 1.2);
    const double kp_final_z = node->declare_parameter<double>("kp_final_z", 0.8);
    const double kp_final_bearing = node->declare_parameter<double>("kp_final_bearing", 1.5);
    const double k_orientation = node->declare_parameter<double>("k_orientation", 0.25);

    const double bearing_center_tol = node->declare_parameter<double>("bearing_center_tol", 0.06);
    const double bearing_approach_break = node->declare_parameter<double>("bearing_approach_break", 0.20);
    const double x_tol = node->declare_parameter<double>("x_tol", 0.015);
    const double z_tol = node->declare_parameter<double>("z_tol", 0.015);
    const double orient_tol = node->declare_parameter<double>("orient_tol", 0.08);
    const int hold_cycles_required = node->declare_parameter<int>("hold_cycles_required", 20);

    const int tag_timeout_ms = node->declare_parameter<int>("tag_timeout_ms", 50);
    const double tag_lost_timeout_sec = node->declare_parameter<double>("tag_lost_timeout_sec", 0.6);

    std::vector<std::string> frame_prefixes = {"tag", "tag36h11:"};
    std::size_t active_prefix_idx = 0;
    aprilTagDetector.setTagFramePrefix(frame_prefixes[active_prefix_idx]);

    auto publishVel = [&](double linear_x, double angular_z) {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = node->now();
        vel.twist.linear.x = linear_x;
        vel.twist.angular.z = angular_z;
        vel_pub->publish(vel);
    };

    auto stopRobot = [&]() {
        publishVel(0.0, 0.0);
    };

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    AlignState state = AlignState::SEARCH;
    AlignState previous_state = state;
    int hold_cycles = 0;

    auto last_tag_time = node->now();
    int active_tag_id = -1;

    rclcpp::Rate rate(loop_hz);

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();                

        std::optional<geometry_msgs::msg::TransformStamped> best_tf = std::nullopt;
        int best_tag_id = -1;
        double best_tag_depth = std::numeric_limits<double>::infinity();

        for (std::size_t prefix_idx = 0; prefix_idx < frame_prefixes.size(); ++prefix_idx) {
            if (active_prefix_idx != prefix_idx) {
                active_prefix_idx = prefix_idx;
                aprilTagDetector.setTagFramePrefix(frame_prefixes[active_prefix_idx]);
            }

            for (int tag_id : candidate_tags) {
                auto tf_opt = aprilTagDetector.getTagTransform(tag_id, tag_timeout_ms);
                if (!tf_opt.has_value()) {
                    continue;
                }

                const double z_depth = tf_opt->transform.translation.z;
                if (z_depth > 0.0 && z_depth < best_tag_depth) {
                    best_tag_depth = z_depth;
                    best_tf = tf_opt;
                    best_tag_id = tag_id;
                }
            }

            if (best_tf.has_value()) {
                break;
            }
        }

        const bool tag_visible = best_tf.has_value();
        if (tag_visible) {
            last_tag_time = node->now();
            active_tag_id = best_tag_id;
        }

        const double seconds_since_tag = (node->now() - last_tag_time).seconds();
        if (!tag_visible && seconds_since_tag > tag_lost_timeout_sec) {
            state = AlignState::SEARCH;
            hold_cycles = 0;
        }

        double cmd_v = 0.0;
        double cmd_w = 0.0;

        if (!tag_visible) {
            cmd_v = 0.0;
            cmd_w = search_angular;
        } else {
            const auto &tr = best_tf->transform.translation;
            const auto &qr = best_tf->transform.rotation;

            const double ex = tr.x - t_des.x;
            const double ez = tr.z - t_des.z;
            const double bearing = std::atan2(tr.x, tr.z);
            const double orient_err = computeOrientationErrorAboutCameraY(qr, q_des);

            switch (state) {
                case AlignState::SEARCH:
                    state = AlignState::CENTER;
                    break;

                case AlignState::CENTER:
                    cmd_v = 0.0;
                    cmd_w = clamp(kp_bearing * bearing, -max_angular, max_angular);

                    if (std::abs(bearing) < bearing_center_tol) {
                        state = AlignState::APPROACH;
                    }
                    break;

                case AlignState::APPROACH:
                    cmd_v = clamp(kp_z * ez, -max_linear, max_linear);
                    cmd_w = clamp(kp_bearing_approach * bearing, -max_angular, max_angular);

                    if (std::abs(bearing) > bearing_approach_break) {
                        state = AlignState::CENTER;
                        hold_cycles = 0;
                    } else if (std::abs(ex) < x_tol && std::abs(ez) < z_tol) {
                        state = AlignState::FINAL_ALIGN;
                    }
                    break;

                case AlignState::FINAL_ALIGN:
                    cmd_v = clamp(kp_final_z * ez, -0.12, 0.12);
                    cmd_w = clamp(
                        kp_final_bearing * bearing + k_orientation * orient_err,
                        -max_angular,
                        max_angular
                    );

                    if (std::abs(ex) < x_tol && std::abs(ez) < z_tol && std::abs(orient_err) < orient_tol) {
                        hold_cycles++;
                        if (hold_cycles >= hold_cycles_required) {
                            state = AlignState::HOLD;
                        }
                    } else {
                        hold_cycles = 0;
                    }
                    break;

                case AlignState::HOLD:
                    cmd_v = 0.0;
                    cmd_w = 0.0;
                    break;
            }

            // If tag gets nearly centered during approach, let heading corrections be smooth.
            if (state == AlignState::APPROACH && std::abs(bearing) < 0.03) {
                cmd_w *= 0.6;
            }

            if (state != AlignState::HOLD) {
                cmd_v = clamp(cmd_v, -max_linear, max_linear);
                cmd_w = clamp(cmd_w, -max_angular, max_angular);
            }

            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "state=%s tag=%d ex=%.3f ez=%.3f bearing=%.3f orient=%.3f cmd_v=%.3f cmd_w=%.3f",
                stateToString(state),
                active_tag_id,
                ex,
                ez,
                bearing,
                orient_err,
                cmd_v,
                cmd_w
            );
        }

        if (state != previous_state) {
            RCLCPP_INFO(node->get_logger(), "State transition: %s -> %s", stateToString(previous_state), stateToString(state));
            previous_state = state;
        }

        publishVel(cmd_v, cmd_w);

        if (state == AlignState::HOLD) {
            RCLCPP_INFO(node->get_logger(), "Alignment converged. Holding pose.");
            break;
        }

        rate.sleep();
    }

    stopRobot();

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
