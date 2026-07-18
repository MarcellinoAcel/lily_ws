#include "lily_navigation/actuator.hpp"
#include "lily_navigation/pid.hpp"
#include "lily_navigation/kinematic.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

// ── Configuration ─────────────────────────────────────────────────────────────

struct BalanceConfig
{
    // IMU topic name — change to match your setup
    std::string imu_topic = "/imu/data";

    // PID gains — tune these for your robot's weight/leg length
    double kp = 2.0;
    double ki = 0.05;
    double kd = 0.3;

    // Target pitch angle (radians). 0 = perfectly upright.
    // Adjust if your robot naturally leans forward/back at rest.
    double target_pitch = 0.0;

    // Maximum joint correction (radians) applied per cycle.
    // Prevents violent overcorrection.
    double max_correction = 0.35;

    // If pitch exceeds this (radians), robot has fallen — stop trying.
    double fall_threshold = 0.8; // ~45 deg

    // Control loop rate
    std::chrono::milliseconds loop_ms{20}; // 50 Hz

    // Joint move duration sent to Actuator each cycle
    double move_duration_s = 0.025;
};

// ── Nominal (upright) pose ────────────────────────────────────────────────────
// These are your baseline joint angles when perfectly balanced.
// The PID correction is *added* to these each cycle.

const std::map<std::string, double> NOMINAL_POSE = {
    {"left_thigh_joint", 2.512},
    {"left_knee_joint", -1.587},
    {"right_thigh_joint", 2.512},
    {"right_knee_joint", -1.587},
};

// ── Helper: quaternion → pitch (radians) ─────────────────────────────────────
double extractPitch(const sensor_msgs::msg::Imu &imu_msg)
{
    tf2::Quaternion q(
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z,
        imu_msg.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return pitch;
}

// ── BalanceNode ───────────────────────────────────────────────────────────────
class BalanceNode : public rclcpp::Node
{
public:
    explicit BalanceNode(Actuator &actuator, const BalanceConfig &cfg)
        : Node("lily_balance_node"),
          actuator_(actuator),
          cfg_(cfg),
          pid_(cfg.kp, cfg.ki, cfg.kd),
          last_time_(this->now()),
          has_imu_(false),
          current_pitch_(0.0),
          fallen_(false)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            cfg_.imu_topic, 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg)
            {
                current_pitch_ = extractPitch(*msg);
                has_imu_ = true;
            });

        control_timer_ = this->create_wall_timer(
            cfg_.loop_ms,
            [this]()
            { controlLoop(); });

        RCLCPP_INFO(this->get_logger(),
                    "BalanceNode ready. IMU: %s | Gains: kp=%.2f ki=%.2f kd=%.2f",
                    cfg_.imu_topic.c_str(), cfg_.kp, cfg_.ki, cfg_.kd);
    }

private:
    void controlLoop()
    {
        if (!has_imu_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 2000, "Waiting for IMU data on '%s'...", cfg_.imu_topic.c_str());
            return;
        }

        // ── Compute dt ────────────────────────────────────────────────────────
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 1.0)
            return; // skip bad deltas

        // ── Fall detection ────────────────────────────────────────────────────
        if (std::abs(current_pitch_) > cfg_.fall_threshold)
        {
            if (!fallen_)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "FALL DETECTED (pitch=%.3f rad). Halting balance loop.", current_pitch_);
                fallen_ = true;
                pid_.reset();
            }
            return;
        }
        if (fallen_)
        {
            // Recovered — reset and resume
            RCLCPP_INFO(this->get_logger(), "Pitch recovered, resuming balance.");
            fallen_ = false;
        }

        // ── PID correction ────────────────────────────────────────────────────
        double error = cfg_.target_pitch - current_pitch_;
        double correction = pid_.compute(error, dt);
        correction = std::clamp(correction, -cfg_.max_correction, cfg_.max_correction);
        double wheel_cmd = correction;
        // ── Apply to joints ───────────────────────────────────────────────────
        // Positive correction (leaning back) → push thighs forward (increase angle).
        // Negative correction (leaning forward) → pull thighs back (decrease angle).
        // Knees stay at nominal to preserve leg geometry.
        std::map<std::string, double> cmd = {
            // {"left_thigh_joint", NOMINAL_POSE.at("left_thigh_joint") + correction},
            // {"right_thigh_joint", NOMINAL_POSE.at("right_thigh_joint") + correction},
            // {"left_knee_joint", NOMINAL_POSE.at("left_knee_joint")},
            // {"right_knee_joint", NOMINAL_POSE.at("right_knee_joint")},

            {"left_wheel_joint", 1},
            {"right_wheel_joint",1}};

        actuator_.moveJoints(cmd,
                             rclcpp::Duration::from_seconds(cfg_.move_duration_s));

        RCLCPP_DEBUG(this->get_logger(),
                     "pitch=%.4f  error=%.4f  correction=%.4f, wheel_cmd=%.4f", current_pitch_, error, correction, wheel_cmd);
    }

    Actuator &actuator_;
    BalanceConfig cfg_;
    PIDController pid_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Time last_time_;
    bool has_imu_;
    double current_pitch_;
    bool fallen_;
};

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    Actuator actuator;
    actuator.waitForServer();

    BalanceConfig cfg;
    // ── Override defaults here or load from ROS params as needed ──────────────
    cfg.imu_topic = "/imu/data";
    cfg.kp = 2.0;
    cfg.ki = 0.05;
    cfg.kd = 0.3;
    cfg.target_pitch = 0.0;
    cfg.max_correction = 0.35;

    auto balance_node = std::make_shared<BalanceNode>(actuator, cfg);

    RCLCPP_INFO(balance_node->get_logger(), "Spinning balance loop...");
    rclcpp::spin(balance_node);

    rclcpp::shutdown();
    return 0;
}