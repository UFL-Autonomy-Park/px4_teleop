#ifndef PX4_TELEOP_HPP
#define PX4_TELEOP_HPP

// Standard Library
#include <map>
#include <set>
#include <vector>

// ROS2
#include <rclcpp/rclcpp.hpp>

// ROS2 Messages
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

// MAVROS Messages & Services
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

// Custom Messages & Libraries
#include "fleet_manager/msg/connected_agents.hpp"
#include <px4_safety_lib/PX4Safety.hpp>
#include "JoyHandler.hpp"

class PX4Teleop : public rclcpp::Node {
public:
    PX4Teleop();

private:
    // === Enums ===
    enum LandedState {
        undefined = 0,
        on_ground,
        in_air,
        takeoff,
        landing
    };

    // === Handlers & Utilities ===
    JoyHandler joy_handler_;
    px4_safety_lib::PX4Safety px4_safety;
    rclcpp::QoS qos_profile_{1};

    // === Persistent Subscriptions ===
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<fleet_manager::msg::ConnectedAgents>::SharedPtr connected_agents_sub_;

    // === Dynamic Subscriptions ===
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_sub_;

    // === Dynamic Publishers ===
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

    // === Service Clients ===
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    // === Agent Management ===
    std::set<std::string> connected_agents_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> cmd_vel_publishers_; // remove, replace with dynamic pub
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>::iterator agent_iterator_; // TODO: iterate over agent list instead of map

    // === State Variables ===
    LandedState landed_state_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::Pose agent_pose_;
    bool pose_init_;

    // === Origin & Coordinate Transformation ===
    double origin_r_;
    double cos_origin_;
    double sin_origin_;

    // === Callback Methods ===
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
    void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);
    void connected_agents_callback(const fleet_manager::msg::ConnectedAgents::SharedPtr connected_agents_msg);

    // === Helper Methods ===
    void initialize_origin_rotation();
    void add_agent(const std::string& agent_name);
    void remove_agent(const std::string& agent_name);
    void switch_agent();
};

#endif // PX4_TELEOP_HPP