#include "LeaderFollower.hpp"

LeaderFollower::LeaderFollower(rclcpp::Node *parent_node)
	: ExperimentRunner{ parent_node }
{
	RCLCPP_INFO(node_->get_logger(), "Hello from Leader Follower Experiment derived class!");
	init_parameters();
	prepare();
	takeoff();
}

void LeaderFollower::init_parameters() {

	node_->declare_parameter("control_rate", 0.0);
	node_->declare_parameter("experiment_id", "");
	node_->declare_parameter("experiment_length", 0.0);

	double cr;
	if (!node_->get_parameter("control_rate", cr)) {
		set_control_rate(cr);
		RCLCPP_ERROR(node_->get_logger(), "Failed to load mission parameter: control rate.");
		rclcpp::shutdown();
	}
	
	std::string id;
	if(!node_->get_parameter("experiment_id", id)) {
		set_experiment_id(id);
		RCLCPP_ERROR(node_->get_logger(), "Failed to load mission parameter: experiment id.");
		rclcpp::shutdown();
	}

	double length;
	if(!node_->get_parameter("experiment_length", length)) {
		set_experiment_length_sec(length);
		RCLCPP_ERROR(node_->get_logger(), "Failed to load mission parameter: experiment length.");
		rclcpp::shutdown();
	}

	RCLCPP_INFO(node_->get_logger(), "Loaded experiment: %s (%.1f sec @ %.1f Hz)", 
					get_experiment_id().c_str(), get_experiment_length_sec(), get_control_rate());
}

bool LeaderFollower::prepare() {
	RCLCPP_INFO(node_->get_logger(), "Called prepare function.");
}

bool LeaderFollower::takeoff() {
	RCLCPP_INFO(node_->get_logger(), "Called takeoff function.");
}

void LeaderFollower::control_loop (
		const geometry_msgs::msg::PoseStamped agent_pose_stamped,
		const geometry_msgs::msg::TwistStamped agent_vel_stamped,
		const std::map<std::string, geometry_msgs::msg::PoseStamped> neighbor_poses_stamped,
		const std::map<std::string, geometry_msgs::msg::TwistStamped> neighbor_vels_stamped
		)
{
	RCLCPP_INFO(node_->get_logger(), "Called control loop.");
}

