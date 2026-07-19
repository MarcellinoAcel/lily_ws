#include <chrono>
#include <map>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Actuator
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    Actuator()
    {
        node_ = std::make_shared<rclcpp::Node>("actuator_node");

        client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/leg_controller/follow_joint_trajectory");

        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&Actuator::jointStateCallback, this, std::placeholders::_1));

        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    rclcpp::Node::SharedPtr getNode()
    {
        return node_;
    }
    // Call this after constructing the Actuator, before sending any goal
    void waitForServer()
    {
        while (!client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "Waiting for action server...");
        }
    }
    void moveJoint(const std::string &joint_name,
                   double position,
                   const rclcpp::Duration &time_from_start)
    {
        cancelActiveGoal();

        FollowJointTrajectory::Goal goal_msg;
        goal_msg.trajectory.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = buildTargetPositions(joint_name, position);
        point.time_from_start = time_from_start;

        goal_msg.trajectory.points.push_back(point);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        options.goal_response_callback =
            [this](GoalHandleFJT::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(node_->get_logger(), "Goal rejected");
                return;
            }

            active_goal_handle_ = goal_handle;
        };

        options.result_callback =
            [this](const GoalHandleFJT::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            }

            active_goal_handle_.reset();
        };

        client_->async_send_goal(goal_msg, options);
    }
    void moveJoints(const std::map<std::string, double> &joint_targets,
                    const rclcpp::Duration &time_from_start)
    {
        cancelActiveGoal();

        FollowJointTrajectory::Goal goal_msg;
        goal_msg.trajectory.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;

        for (const auto &name : joint_names_)
        {
            if (joint_targets.count(name))
                point.positions.push_back(joint_targets.at(name));
            else if (joint_positions_.count(name))
                point.positions.push_back(joint_positions_.at(name));
            else
                point.positions.push_back(0.0);
        }

        point.time_from_start = time_from_start;
        goal_msg.trajectory.points.push_back(point);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        options.goal_response_callback =
            [this](GoalHandleFJT::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(node_->get_logger(), "Goal rejected");
                return;
            }
            active_goal_handle_ = goal_handle;
        };

        options.result_callback =
            [this](const GoalHandleFJT::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");

            active_goal_handle_.reset();
        };

        client_->async_send_goal(goal_msg, options);
    }

private:
    const std::vector<std::string> joint_names_{
        "left_knee_joint",
        "left_thigh_joint",
        "right_knee_joint",
        "right_thigh_joint"};

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::map<std::string, double> joint_positions_;
    GoalHandleFJT::SharedPtr active_goal_handle_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            joint_positions_[msg->name[i]] = msg->position[i];
        }
    }

    std::vector<double> buildTargetPositions(const std::string &joint_name,
                                             double target_position)
    {
        std::vector<double> positions;

        for (const auto &name : joint_names_)
        {
            if (name == joint_name)
            {
                positions.push_back(target_position);
            }
            else
            {
                if (joint_positions_.count(name))
                    positions.push_back(joint_positions_[name]);
                else
                    positions.push_back(0.0);
            }
        }

        return positions;
    }

    void cancelActiveGoal()
    {
        if (!active_goal_handle_)
            return;

        client_->async_cancel_goal(active_goal_handle_);
        active_goal_handle_.reset();
    }
};