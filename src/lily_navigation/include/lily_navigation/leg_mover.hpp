#include <chrono>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class LegTrajectoryActionClient
{
public:
    explicit LegTrajectoryActionClient()
    {
        node_ = std::make_shared<rclcpp::Node>("leg_trajectory_action_client");
        client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/leg_controller/follow_joint_trajectory");

        if (!client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&LegTrajectoryActionClient::joint_state_callback, this, std::placeholders::_1));
    }
    rclcpp::Node::SharedPtr get_node()
    {
        return node_;
    }
    bool has_joint_states() const
    {
        return joint_positions_.count("left_knee_joint") &&
               joint_positions_.count("left_thigh_joint") &&
               joint_positions_.count("right_knee_joint") &&
               joint_positions_.count("right_thigh_joint");
    }

    void move_joints(const std::map<std::string, double> &joint_targets,
                     const rclcpp::Duration &time_from_start)
    {
        const std::vector<std::string> all_joints = {
            "left_knee_joint", "left_thigh_joint",
            "right_knee_joint", "right_thigh_joint"};

        // Ensure all current joint states are available
        for (const auto &name : all_joints)
        {
            if (joint_positions_.find(name) == joint_positions_.end())
            {
                RCLCPP_ERROR(node_->get_logger(), "Joint '%s' state not yet received.", name.c_str());
                return;
            }
        }

        // Validate that every requested joint actually exists
        for (const auto &[joint_name, _] : joint_targets)
        {
            if (joint_positions_.find(joint_name) == joint_positions_.end())
            {
                RCLCPP_ERROR(node_->get_logger(), "Requested joint '%s' not found in joint states.", joint_name.c_str());
                return;
            }
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = all_joints;

        // Use target position if specified, otherwise hold current position
        std::vector<double> target_positions;
        for (const auto &name : all_joints)
        {
            auto it = joint_targets.find(name);
            if (it != joint_targets.end())
            {
                RCLCPP_INFO(node_->get_logger(), "Moving '%s' to %.2f in %.2f seconds",
                            name.c_str(), it->second, time_from_start.seconds());
                target_positions.push_back(it->second);
            }
            else
            {
                target_positions.push_back(joint_positions_.at(name));
            }
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions;
        point.time_from_start = time_from_start;
        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleFJT::SharedPtr &goal_handle)
        {
            if (!goal_handle)
                RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
            else
                RCLCPP_INFO(node_->get_logger(), "Goal accepted, waiting for result...");
        };

        send_goal_options.feedback_callback =
            [this](GoalHandleFJT::SharedPtr,
                   const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
        {
            if (!feedback->error.positions.empty())
                RCLCPP_INFO(node_->get_logger(), "Feedback: error = %.4f", feedback->error.positions[0]);
        };

        send_goal_options.result_callback =
            [this](const GoalHandleFJT::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(node_->get_logger(), "Goal was canceled.");
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
                break;
            }
            rclcpp::shutdown();
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }
    void move_joint(const std::string &joint_name, double position,
                    const rclcpp::Duration &time_from_start)
    {
        const std::vector<std::string> all_joints = {
            "left_knee_joint", "left_thigh_joint",
            "right_knee_joint", "right_thigh_joint"};
        for (const auto &name : all_joints)
        {
            if (joint_positions_.find(name) == joint_positions_.end())
            {
                RCLCPP_ERROR(node_->get_logger(), "Joint '%s' state not yet received.", name.c_str());
                return;
            }
        }
        if (joint_positions_.find(joint_name) == joint_positions_.end())
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found in joint states.", joint_name.c_str());
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();

        goal_msg.trajectory.joint_names = {
            "left_knee_joint",
            "left_thigh_joint",
            "right_knee_joint",
            "right_thigh_joint"};

        // Keep all joints at current position, only move the target joint
        std::vector<double> target_positions;
        for (const auto &name : goal_msg.trajectory.joint_names)
        {
            if (name == joint_name)
                target_positions.push_back(position);
            else
                target_positions.push_back(joint_positions_.at(name));
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions;
        point.time_from_start = time_from_start;
        goal_msg.trajectory.points.push_back(point);

        RCLCPP_INFO(node_->get_logger(), "Moving '%s' to %.2f in %.2f seconds",
                    joint_name.c_str(), position, time_from_start.seconds());

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleFJT::SharedPtr &goal_handle)
        {
            if (!goal_handle)
                RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
            else
                RCLCPP_INFO(node_->get_logger(), "Goal accepted, waiting for result...");
        };

        send_goal_options.feedback_callback =
            [this](GoalHandleFJT::SharedPtr,
                   const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
        {
            if (!feedback->error.positions.empty())
                RCLCPP_INFO(node_->get_logger(), "Feedback: error = %.4f", feedback->error.positions[0]);
        };

        send_goal_options.result_callback =
            [this](const GoalHandleFJT::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(node_->get_logger(), "Goal was canceled.");
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
                break;
            }
            rclcpp::shutdown();
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::map<std::string, double> joint_positions_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
            joint_positions_[msg->name[i]] = msg->position[i];
    }
};