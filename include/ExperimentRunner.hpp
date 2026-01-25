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

private:
	virtual ~ExperimentRunner();	

	// implemented by each experiment
	virtual bool prepare(/* params here */) = 0;
	virtual bool takeoff(/* params here */) = 0;
	virtual void controlLoop (
		const geometry_msgs::msg::PoseStamped agent_pose_stamped,
		const geometry_msgs::msg::TwistStamped agent_vel_stamped,
		const std::map<std::string, geometry_msgs::msg::PoseStamped> neighbor_poses_stamped,
		const std::map<std::string, geometry_msgs::msg::TwistStamped> neighbor_vels_stamped
	) = 0;

	// common functions implemented by base class
	virtual double get_control_rate() const {return control_rate_};
	virtual std::string get_id() const {return experiment_id_};
	virtual double get_experiment_length() {return experimnet_length_sec_};

	virtual void start();
	virtual void stop();


	double control_rate_
	std::string experiment_id_;
	double experiment_length_sec_;
};

#endif
