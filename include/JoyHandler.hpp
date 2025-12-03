#ifndef JOY_HANDLER_HPP
#define JOY_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class JoyHandler {
    public:

    struct joy_action {
        // mode switching (buttons)
        bool arm;
        bool disarm;
        bool switch_agent;
        bool offboard;
		bool takeoff;
        
        // movement commands (joystick axes)
        double linear_x;
        double linear_y;
        double linear_z;
        double angular_z;
    };

    joy_action process(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

    JoyHandler(rclcpp::Node *parent_node);

private:

    rclcpp::Node *node_;
    struct Axis {
        Axis() : axis(0), factor(0.0), offset(0.0) {}

        int axis;
        double factor;
        double offset;
    };

    struct {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    } axes_ ;

    struct Button {
        Button() : index(0) {}
        int index;
    };

    struct {
        Button arm;
        Button disarm;
        Button offboard;
        Button follow;
        Button control;
		Button takeoff;
    } buttons_;

    // button state for debouncing
    struct ButtonState {
        ButtonState() : state(0) {}
        int state;
    };

    struct {
        ButtonState arm;
        ButtonState disarm;
        ButtonState offboard;
        ButtonState follow;
        ButtonState control;
        ButtonState switchAgent;
		ButtonState takeoff;
    } button_state_;

    double get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis);
    int get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button);
    void init_parameters();
};
#endif
