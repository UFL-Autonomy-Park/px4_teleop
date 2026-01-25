#include "ExperimentRunner.hpp"

ExperimentRunner::ExperimentRunner(rclcpp::Node *parent_node) {
	node_ = parent_node;

	init_parameters();
}

void ExperimentRunner::init_parameters() {

	node_->declare_parameter("control_rate", 0.0);
	node_->declare_parameter("experiment_id", "");
	node_->declare_parameter("experiment_length", 0.0);

	if (!node_->get_parameter("control_rate", control_rate)) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to load mission parameter: control rate.");
		rclcpp::shutdown();
	}

	if(!node_->get_parameter("experiment_id", experiment_id)) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to load mission parameter: experiment id.");
		rclcpp::shutdown();
	}

	if(!node_->get_parameter("experiment_length", experiment_length_sec)) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to load mission parameter: experiment length.");
		rclcpp::shutdown();
	}

	RCLCPP_INFO(node_->get_logger(), "Loaded experiment: %s (%.1f sec @ %.1f Hz)", 
					experiment_id_.c_str(), experiment_length_sec_, control_rate_);
	}
}

void ExperimentRunner::start() {
	/* start is called after agent takeoff
	   intuitively agents will just go offboard
	   and execute their control loops?
	 */
}

void ExperimentRunner::stop() {
	/* Some landing procedure here */
}

ExperimentRunner::~ExperimentRunner() = default;
