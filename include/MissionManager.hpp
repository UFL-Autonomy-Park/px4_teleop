#ifndef MISSION_MANAGER_HPP
#define MISSION_MANAGER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class MissionManager {
public:
	MissionManager();

private:
	virtual ~MissionManager();	

	// lifecycle 
	virtual int get_mission_id() = 0;
	virtual bool prepare(/* params here */) = 0;
	virtual bool takeoff(/* params here */) = 0;
	virtual void start() = 0;
	virtual void stop() = 0;

	// control
	virtual double get_control_rate() = 0;
	virtual void controlLoop (
		const geometry_msgs::msg::PoseStamped agent_pose_stamped,
		const geometry_msgs::msg::TwistStamped agent_vel_stamped,
		const std::map<std::string, geometry_msgs::msg::PoseStamped> neighbor_poses_stamped,
		const std::map<std::string, geometry_msgs::msg::TwistStamped> neighbor_vels_stamped
	) = 0;
		
};

#endif
