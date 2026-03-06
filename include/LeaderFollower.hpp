#ifndef LEADER_FOLLOWER_HPP_
#define LEADER_FOLLOWER_HPP_

#include "ExperimentRunner.hpp"

class LeaderFollower : public ExperimentRunner {
public:
	LeaderFollower(rclcpp::Node *parent_node);

	bool prepare() override;
	bool takeoff() override;
	void control_loop (
		const geometry_msgs::msg::PoseStamped agent_pose_stamped,
		const geometry_msgs::msg::TwistStamped agent_vel_stamped,
		const std::map<std::string, geometry_msgs::msg::PoseStamped> neighbor_poses_stamped,
		const std::map<std::string, geometry_msgs::msg::TwistStamped> neighbor_vels_stamped
	) override;
	void init_parameters() override;
};

#endif // LEADER_FOLLOWER_HPP_

