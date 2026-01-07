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

    enum LandedState {
        undefined = 0,
        on_ground,
        in_air,
        takeoff,
        landing
    };

	struct neighborState {
		geometry_msgs::msg::Pose pose;
		geometry_msgs::msg::Twist velocity;
		mavros_msgs::msg::State state;
		LandedState landed_state;
		rclcpp::Time last_update;
	};

    // === Handlers & Utilities ===
    JoyHandler joy_handler_;
    px4_safety_lib::PX4Safety px4_safety;


	// pub subs
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<swarm_interfaces::msg::ConnectedAgents>::SharedPtr connected_agents_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpos_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_agent_sub_;

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
	//
	std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> neighbor_pose_subscriptions_;
	std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> neighbor_velocity_subscriptions_;
	std::map<std::string, rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr> neighbor_state_subscriptions_;
	std::map<std::string, rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr> neighbor_ext_state_subscriptions_;
	std::map<std::string, geometry_msgs::msg::PoseStamped> neighbor_poses_;
	std::map<std::string, geometry_msgs::msg::TwistStamped> neighbor_velocities_;
	std::map<std::string, mavros_msgs::msg::State> neighbor_states_;
	std::map<std::string, mavros_msgs::msg::ExtendedState> neighbor_ext_states_;

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

	// mission parameters
	std::string mission_id_;
	float takeoff_height_;
	uint32_t mission_start_time_;
	float minimum_takeoff_separation_;
	// mission pub/sub
	rclcpp::Publisher<swarm_interfaces::msg::PrepareMissionResponse>::SharedPtr pmr_pub_;
	rclcpp::Publisher<swarm_interfaces::msg::InitiateTakeoffResponse>::SharedPtr itr_pub_;
	rclcpp::Subscription<swarm_interfaces::msg::PrepareMissionCommand>::SharedPtr pmc_sub_;
	rclcpp::Subscription<swarm_interfaces::msg::InitiateTakeoffCommand>::SharedPtr itc_sub_;
	rclcpp::Subscription<swarm_interfaces::msg::StartMissionCommand>::SharedPtr smc_sub_;

	// agent callbacks
	void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr altitude_msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
    void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);
    void connected_agents_callback(const swarm_interfaces::msg::ConnectedAgents::SharedPtr connected_agents_msg);
    void global_gpos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void loiter_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
	void active_agent_callback(const std_msgs::msg::String::SharedPtr active_agent);
	void tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future);
	void arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);

	// mission callbacks
	void pmc_callback(const swarm_interfaces::msg::PrepareMissionCommand::SharedPtr pmc_msg);
	void itc_callback(const swarm_interfaces::msg::InitiateTakeoffCommand::SharedPtr itc_msg);
	void smc_callback(const swarm_interfaces::msg::StartMissionCommand::SharedPtr smc_msg);

	// neighbor callbacks
	void neighbor_pose_callback(const geometry_msgs::msg::Pose::SharedPtr neighbor_pose_msg);
	void neighbor_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr neighbor_vel_msg);
	void neighbor_state_callback(const mavros_msgs::msg::State::SharedPtr neighbor_state_msg);
	void neighbor_ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr neighbor_ext_state_msg);

    // initialization
	void init_origin_rotation();
	void init_subscribers();
	void init_publishers();
	void init_service_clients();

	void send_tol_request(bool takeoff);
	double quat_to_yaw(geometry_msgs::msg::Quaternion quat);
    void add_agent(const std::string& agent_name);
    void remove_agent(const std::string& agent_name);
	void send_arming_request(bool arm);

    inline float compute_horizontal_separation(const geometry_msgs::msg::Point p1, const geometry_msgs::msg::Point p2)
	{
		return std::hypot(p1.x - p2.x, p1.y - p2.y);
	}
};

#endif // PX4_TELEOP_HPP
