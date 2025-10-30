#ifndef PX4_TELEOP_HPP
#define PX4_TELEOP_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <px4_safety_lib/PX4Safety.hpp>

#include "fleet_manager/msg/connected_agents.hpp"
#include "JoyHandler.hpp"
#include <vector>
#include <set>
#include <map>

class PX4Teleop : public rclcpp::Node {
private:

    JoyHandler joy_handler_;
    rclcpp::QoS qos_profile_{1};
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
    rclcpp::Subscription<fleet_manager::msg::ConnectedAgents>::SharedPtr connected_agents_sub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    enum LandedState {
        undefined = 0,
        on_ground,
        in_air,
        takeoff,
        landing
    };

    LandedState landed_state_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::Pose agent_pose_;
    bool switch_agent_state_{false};
    int current_agent_index_{0};
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>::iterator agent_iterator_;
    void add_agent(const std::string& agent_name);
    void remove_agent(const std::string& agent_name);
    std::set<std::string> connected_agents_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> cmd_vel_publishers_;

    double origin_r_, cos_origin_, sin_origin_;

    px4_safety_lib::PX4Safety px4_safety;
    bool pose_init_;

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
	void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
	void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);
    void connected_agents_callback(const fleet_manager::msg::ConnectedAgents::SharedPtr connected_agents_msg);

    void initialize_origin_rotation();

public:
    PX4Teleop();
};

#endif
