#ifndef PX4_TELEOP_HPP
#define PX4_TELEOP_HPP

// Standard Library
#include <map>
#include <set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

// MAVROS Messages & Services
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

// Custom Messages & Libraries
#include "swarm_interfaces/msg/prepare_mission_command.hpp"
#include "swarm_interfaces/msg/prepare_mission_response.hpp"
#include "swarm_interfaces/msg/initiate_takeoff_command.hpp"
#include "swarm_interfaces/msg/initiate_takeoff_response.hpp"
#include "swarm_interfaces/msg/start_mission_command.hpp"
#include "swarm_interfaces/msg/connected_agents.hpp"
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

    // === Persistent Subscriptions ===
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<swarm_interfaces::msg::ConnectedAgents>::SharedPtr connected_agents_sub_;

    // === Dynamic Subscriptions ===
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpos_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_agent_sub_;

    // === Dynamic Publishers ===
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

    // === Service Clients ===
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    // === Agent Management ===
	std::string px4_id_;
	std::string active_agent_id_;
    std::set<std::string> connected_agents_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> cmd_vel_publishers_; // remove, replace with dynamic pub
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>::iterator agent_iterator_; // TODO: iterate over agent list instead of map

    // === State Variables ===
    LandedState landed_state_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::Pose apark_pose_;
    geographic_msgs::msg::GeoPose global_pose_;

	double altitude_amsl_;

	bool gpos_init_;
    bool pose_init_;
	bool landing_requested_;
	bool sim_mode_;
	bool alt_init_;

    // === Origin & Coordinate Transformation ===
    double origin_r_;
    double cos_origin_;
    double sin_origin_;

	// swarm interface
	rclcpp::Publisher<swarm_interfaces::msg::PrepareMissionResponse>::SharedPtr pmr_pub_;
	rclcpp::Subscription<swarm_interfaces::msg::PrepareMissionCommand>::SharedPtr pmc_sub_;

	rclcpp::Publisher<swarm_interfaces::msg::InitiateTakeoffResponse>::SharedPtr itr_pub_;
	rclcpp::Subscription<swarm_interfaces::msg::InitiateTakeoffCommand>::SharedPtr itc_sub_;

	rclcpp::Subscription<swarm_interfaces::msg::StartMissionCommand>::SharedPtr smc_sub_;

    // === Callback Methods ===
	void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr altitude_msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
    void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);
    void connected_agents_callback(const swarm_interfaces::msg::ConnectedAgents::SharedPtr connected_agents_msg);
    void global_gpos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void loiter_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
	void active_agent_callback(const std_msgs::msg::String::SharedPtr active_agent);
	void pmc_callback(const swarm_interfaces::msg::PrepareMissionCommand::SharedPtr pmc_msg);
	void itc_callback(const swarm_interfaces::msg::InitiateTakeoffCommand::SharedPtr itc_msg);
	void smc_callback(const swarm_interfaces::msg::StartMissionCommand::SharedPtr smc_msg);

    // === Helper Methods ===
    void init_origin_rotation();
	void init_subscribers();
	void init_publishers();
	void init_service_clients();
	void send_tol_request(bool takeoff);
	double quat_to_yaw(geometry_msgs::msg::Quaternion quat);
	void tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future);

    void add_agent(const std::string& agent_name);
    void remove_agent(const std::string& agent_name);
    void switch_agent();

};

#endif // PX4_TELEOP_HPP
