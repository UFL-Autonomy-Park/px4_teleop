#ifndef JOY_HANDLER_HPP
#define JOY_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class JoyHandler {
private:
    struct Axis {
        Axis() : axis(0), factor(0.0), offset(0.0) {}

        int axis;
        double factor;
        double offset;
    };

    struct axes_ {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    };

    double get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis);

public:
    struct joy_action {
        bool land;
        bool switch_agent;
        bool offboard;
        bool arm;
        double linear_x;
        double linear_y;
        double linear_z;
        double angular_z;
    };

    joy_action process(const sensor_msgs::msg::Joy &msg);

    JoyHandler(rclcpp::Node::SharedPtr teleop_node);
};

#endif