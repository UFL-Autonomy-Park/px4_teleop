#include "JoyHandler.hpp"

JoyHandler::JoyHandler(rclcpp::Node *parent_node) {
    node_ = parent_node;

    init_parameters();
}       

void JoyHandler::init_parameters() {
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
        RCLCPP_ERROR(node_->get_logger(), "Joy parameters failed to load. Exiting.");
        rclcpp::shutdown();
    }

    // init button mappings
    this->declare_parameter("arm_button", -1);
    this->declare_parameter("disarm_button", -1);
    this->declare_parameter("control_button", -1);
    this->declare_parameter("follow_setpoint_button", -1);

    if (this->get_parameter("arm_button", buttons_.arm.button) &&
        this->get_parameter("disarm_button", buttons_.disarm.button) &&
        this->get_parameter("control_button", buttons_.control.button) &&
        this->get_parameter("follow_setpoint_button", buttons_.follow.button))
    {
        RCLCPP_INFO(this->get_logger(), "Loaded button mappings:\nArm: %d, Disarm: %d, Control: %d, Follow: %d", buttons_.arm.index, buttons_.disarm.index, buttons_.control.index, buttons_.follow.index);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Button parameters failed to load. Exiting.");
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
    int switch_agent_button_state = get_button(joy_msg, buttons_.control);
    if (switch_agent_button_state != button_state_.switchAgent.state) {
        if (switch_agent_button_state == 1) {
            RCLCPP_INFO(node_->get_logger(), "Switch agent button pressed.");
            action.switch_agent = true;
        } else {
            action.switch_agent = false;
        }
        button_state_.switchAgent.state = switch_agent_button_state;
    } else {
        action.switch_agent = false;
    }

    // check arm button
    int arm_button_state = get_button(joy_msg, buttons_.arm);
    if (arm_button_state != button_state_.arm.state) {
        if (arm_button_state == 1) {
            RCLCPP_INFO(node_->get_logger(), "Arm button pressed.");
            action.arm = true;
        } else {
            action.arm = false;
        }
        button_state_.arm.state = arm_button_state;
    } else {
        action.arm = false;
    }

    // check disarm button
    int disarm_button_state = get_button(joy_msg, buttons_.disarm);
    if (disarm_button_state != button_state_.disarm.state) {
        if (disarm_button_state == 1) {
            RCLCPP_INFO(node_->get_logger(), "Disarm button pressed.");
            action.disarm = true;
        } else {
            action.disarm = false;
        }
        button_state_.disarm.state = disarm_button_state;
    } else {
        action.disarm = false;

        // TODO: ADDITIONAL LOGIC NEEDED HERE TO DIFFERENTIATE LAND VS DISARM + AUTO LOITER STUFF?
    }

    // TODO: HANDLE OFFBOARD BUTTON PRESS, SWITCH BTWN OFFBOARD AND AUTO LOITER MODES
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

int JoyHandler::get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button) {
    if (button.index < 0 || button.index > (int)joy_msg->buttons.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Button %d out of range, joy has %d buttons", button.index, (int)joy_msg->buttons.size());
        return -1;
    }

    return joy_msg->buttons[button.index];
}