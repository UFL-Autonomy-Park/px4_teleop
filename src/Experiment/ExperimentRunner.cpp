#include "ExperimentRunner.hpp"

ExperimentRunner::ExperimentRunner(rclcpp::Node *parent_node)
	: node_{ parent_node }
{
	RCLCPP_INFO(node_->get_logger(), "Hello from base Experiment Runner class!");
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


double get_control_rate() { return control_rate_; }
std::string get_experiment_id() { return experiment_id_; }
double get_experiment_length_sec() {return experiment_length_sec_; }

void set_control_rate(double rate) { control_rate_ = rate; }
void set_experiment_id(std::string id) { experiment_id_ = id; }
void set_experiment_length_sec(doublde length) { experiment_length_sec_ = length; } 
