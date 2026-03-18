#include "lily_navigation/actuator.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

// ── Poses ─────────────────────────────────────────────────────────────────────

// TODO: confirm left_thigh_joint value (using -2.512 as mirror of right for now)
const std::map<std::string, double> STAND_POSE = {
    {"left_thigh_joint", 2.512},
    {"left_knee_joint", -1.587},
    {"right_thigh_joint", 2.512},
    {"right_knee_joint", -1.587}};

// Gait: alternate between phase A and B while driving
const std::map<std::string, double> GAIT_A = {
    {"left_thigh_joint", 2.812}, // left leg forward
    {"left_knee_joint", -1.587},
    {"right_thigh_joint", 2.212}, // right leg back
    {"right_knee_joint", -1.587}};

const std::map<std::string, double> GAIT_B = {
    {"left_thigh_joint", 2.212}, // left leg back
    {"left_knee_joint", -1.587},
    {"right_thigh_joint", 2.812}, // right leg forward
    {"right_knee_joint", -1.587}};

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    Actuator actuator;
    actuator.waitForServer();

    while (rclcpp::ok())
    {

        actuator.moveJoints(
            {{"left_knee_joint", -1.587},
             {"right_knee_joint", -1.587}},
            rclcpp::Duration::from_seconds(0.1));

        rclcpp::spin_some(actuator.getNode()); // process callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}