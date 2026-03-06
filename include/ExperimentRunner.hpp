#ifndef EXPERIMENT_RUNNER_HPP
#define EXPERIMENT_RUNNER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>

class ExperimentRunner {
public:
	ExperimentRunner(rclcpp::Node *parent_node);

protected:
	rclcpp::Node *node_;
	void set_control_rate(double rate);
	void set_experiment_id(std::string id);
	void set_experiment_length_sec(double length); 

private:
	// implemented by each experiment
	virtual bool prepare(/* params here */) = 0;
	virtual bool takeoff(/* params here */) = 0;
	virtual void control_loop (
		const geometry_msgs::msg::PoseStamped agent_pose_stamped,
		const geometry_msgs::msg::TwistStamped agent_vel_stamped,
		const std::map<std::string, geometry_msgs::msg::PoseStamped> neighbor_poses_stamped,
		const std::map<std::string, geometry_msgs::msg::TwistStamped> neighbor_vels_stamped
	) = 0;
	virtual void init_parameters() = 0;

	// common functions implemented by base class
	double get_control_rate() const {return control_rate_;};
	std::string get_id() const {return experiment_id_;};
	double get_experiment_length() {return experiment_length_sec_;};

	void start();
	void stop();


	double control_rate_;
	std::string experiment_id_;
	double experiment_length_sec_;

	double get_control_rate();
	std::string get_experiment_id();
	double get_experiment_length_sec();
};

#endif
