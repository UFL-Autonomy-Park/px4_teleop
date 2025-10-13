#include "JoyHandler.hpp"

JoyHandler::JoyHandler(rclcpp::Node *parent_node) {
    node_ = parent_node;
    
    //Get controller parameters
    node_->declare_parameter("x_axis", -1);
    node_->declare_parameter("y_axis", -1);
    node_->declare_parameter("z_axis", -1);
    node_->declare_parameter("yaw_axis", -1);

    node_->declare_parameter("x_vel_max", 1.0);
    node_->declare_parameter("y_vel_max", 1.0);
    node_->declare_parameter("z_vel_max", 1.0);
    node_->declare_parameter("yaw_vel_max", 1.0);

    if(node_->get_parameter("x_axis", axes_.x.axis) &&
        node_->get_parameter("y_axis", axes_.y.axis) &&
        node_->get_parameter("z_axis", axes_.z.axis) &&
        node_->get_parameter("yaw_axis", axes_.yaw.axis) &&
        node_->get_parameter("x_vel_max", axes_.x.factor) &&
        node_->get_parameter("y_vel_max", axes_.y.factor) &&
        node_->get_parameter("z_vel_max", axes_.z.factor) &&
        node_->get_parameter("yaw_vel_max", axes_.yaw.factor))
    {
        RCLCPP_INFO(node_->get_logger(), "Loaded controller axis parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d", axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis);
    }

    else {
        RCLCPP_ERROR(node_->get_logger(), "Park origin rotation not set. Exiting.");
        rclcpp::shutdown();
    }
}

JoyHandler::joy_action JoyHandler::process(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    joy_action action;

    // get joystick axes
    action.linear_x = get_axis(joy_msg, axes_.x);
    action.linear_y = get_axis(joy_msg, axes_.y);
    action.linear_z = get_axis(joy_msg, axes_.z);
    action.angular_z = get_axis(joy_msg, axes_.yaw);

    // check switch agent button 
    if (joy_msg->buttons[5] == 1 && pressed_buttons_.switch_agent == false) {
        action.switch_agent = true;
        pressed_buttons_.switch_agent = true;
    }
    else if (joy_msg->buttons[5] == 0 && pressed_buttons_.switch_agent == true) {
        action.switch_agent = false;
        pressed_buttons_.switch_agent = false;
    }

    // handle other buttons (currently in telemetry node)
    return action;
}


double JoyHandler::get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis) {
    if (axis.axis < 0 || std::abs(axis.axis) > (int)joy_msg->axes.size()-1) {
        RCLCPP_ERROR(node_->get_logger(), "Axis %d out of range, joy has %d axes", axis.axis, (int)joy_msg->axes.size());
        return -1;
    }

    double output = joy_msg->axes[std::abs(axis.axis)] * axis.factor + axis.offset;

    return output;
}