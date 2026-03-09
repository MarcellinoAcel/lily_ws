#include <cstdio>
#include <lily_navigation/leg_mover.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<LegTrajectoryActionClient>();

    rclcpp::Rate rate(10);

    // wait until joint states are received
    while (rclcpp::ok() && !client->has_joint_states())
    {
        rclcpp::spin_some(client->get_node());
        rate.sleep();
    }
    client->move_joints(
        {{"left_knee_joint", -1.57},
         {"right_knee_joint", -1.57},
         {"left_thigh_joint", 2.53},
         {"right_thigh_joint", 2.53}},
        rclcpp::Duration::from_seconds(2.0));

    rclcpp::spin(client->get_node());

    return 0;
}