#include "JoyHandler.hpp"

JoyHandler::JoyHandler(rclcpp::Node::SharedPtr teleop_node) {
    //Get controller parameters
    teleop_node->declare_parameter("x_axis", -1);
    teleop_node->declare_parameter("y_axis", -1);
    teleop_node->declare_parameter("z_axis", -1);
    teleop_node->declare_parameter("yaw_axis", -1);

    teleop_node->declare_parameter("x_vel_max", 1.0);
    teleop_node->declare_parameter("y_vel_max", 1.0);
    teleop_node->declare_parameter("z_vel_max", 1.0);
    teleop_node->declare_parameter("yaw_vel_max", 1.0);

    if(teleop_node->get_parameter("x_axis", axes_.x.axis) &&
        teleop_node->get_parameter("y_axis", axes_.y.axis) &&
        teleop_node->get_parameter("z_axis", axes_.z.axis) &&
        teleop_node->get_parameter("yaw_axis", axes_.yaw.axis) &&
        teleop_node->get_parameter("x_vel_max", axes_.x.factor) &&
        teleop_node->get_parameter("y_vel_max", axes_.y.factor) &&
        teleop_node->get_parameter("z_vel_max", axes_.z.factor) &&
        teleop_node->get_parameter("yaw_vel_max", axes_.yaw.factor))
    {
        RCLCPP_INFO(teleop_node->get_logger(), "Loaded controller axis parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d", axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis);
    }

    else {
        RCLCPP_ERROR(teleop_node->get_logger(), "Park origin rotation not set. Exiting.");
        rclcpp::shutdown();
    }
}

geometry_msgs::msg::TwistStamped JoyHandler::process(const sensor_msgs::msg::Joy &msg) {

}