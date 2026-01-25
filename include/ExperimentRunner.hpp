#ifndef EXPERIMENT_RUNNER_HPP
#define EXPERIMENT_RUNNER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <map>

class MissionManager {
public:
	MissionManager();

private:
	virtual ~MissionManager();	

	// lifecycle 
	virtual int get_mission_id();
	virtual bool prepare(/* params here */);
	virtual bool takeoff(/* params here */);
	virtual void start();
	virtual void stop();

	// control
	virtual double get_control_rate();
	virtual void controlLoop (
		const geometry_msgs::msg::PoseStamped agent_pose_stamped,
		const geometry_msgs::msg::TwistStamped agent_vel_stamped,
		const std::map<std::string, geometry_msgs::msg::PoseStamped> neighbor_poses_stamped,
		const std::map<std::string, geometry_msgs::msg::TwistStamped> neighbor_vels_stamped
	) = 0;
		
};

#endif
