#include "PX4Teleop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PX4Teleop::PX4Teleop() : Node("px4_teleop_node"), joy_handler_(this), pose_init_(false) {
	RCLCPP_INFO(this->get_logger(), "Initializing PX4 Teleop Node");

    //Get park rotation from params (not optimal but needed because vel commands go to global ENU frame)
    if(!initialize_origin_rotation()) rclcpp::shutdown();

    // initialize PX4 safety library
    px4_safety.initialize(this);

    // publish setpoints at 50Hz
    setpoint_timer_ = this->create_wall_timer(20ms, std::bind(&PX4Teleop::publish_setpoint, this));

    // setup qos profile and subscribers
    qos_profile_.reliable();
    qos_profile_.transient_local(); // latched topics
    connected_agents_sub_ = this->create_subscription<fleet_manager::msg::ConnectedAgents>("/connected_agents", qos_profile_, std::bind(&PX4Teleop::connected_agents_callback, this, _1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PX4Teleop::joy_callback, this, _1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/astro_sim1/autonomy_park/pose", 10, std::bind(&PX4Teleop::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "PX4 Teleop Initialized.");
}

void PX4Teleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (!pose_init_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring joy inputs until agent pose is initialized.");
        return;
    }

    // process joy message with joy handler (returns struct with actions)
    JoyHandler::joy_action action = joy_handler_.process(joy_msg);
    
    // check for switch agent action
    if(action.switch_agent == true && !cmd_vel_publishers_.empty()) {
        agent_iterator_++;

        if(agent_iterator_ == cmd_vel_publishers_.end())
            agent_iterator_ = cmd_vel_publishers_.begin();

        setpoint_vel_.header.frame_id = agent_iterator_->first;
        RCLCPP_INFO(this->get_logger(), "controlling agent: %s", agent_iterator_->first.c_str());
    }

    // store unfiltered velocity command
    geometry_msgs::msg::Twist unsafe_cmd_vel;
    unsafe_cmd_vel.linear.x = action.linear_x;
    unsafe_cmd_vel.linear.y = action.linear_y;
    unsafe_cmd_vel.linear.z = action.linear_z;
    unsafe_cmd_vel.angular.z = action.angular_z;

    //Generate safe velocity command
    geometry_msgs::msg::Twist safe_cmd_vel = px4_safety.compute_safe_cmd_vel(agent_pose_, unsafe_cmd_vel);

    //Convert autonomy park X/Y velocity command to ENU frame
    setpoint_vel_.twist.linear.x = cos_origin_*safe_cmd_vel.linear.x + sin_origin_*safe_cmd_vel.linear.y;
    setpoint_vel_.twist.linear.y = -sin_origin_*safe_cmd_vel.linear.x + cos_origin_*safe_cmd_vel.linear.y;
    setpoint_vel_.twist.linear.z = safe_cmd_vel.linear.z;
    setpoint_vel_.twist.angular.z = safe_cmd_vel.angular.z;
}

void PX4Teleop::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    //Update PX4 agent pose
    agent_pose_ = pose_msg->pose;

    if (!pose_init_) pose_init_ = true;
}

void PX4Teleop::publish_setpoint() {
    rclcpp::Time now = this->get_clock()->now();
    setpoint_vel_.header.stamp = this->get_clock()->now();
    if(!cmd_vel_publishers_.empty())
        agent_iterator_->second->publish(setpoint_vel_);
}


void PX4Teleop::connected_agents_callback(const fleet_manager::msg::ConnectedAgents::SharedPtr connected_agents_msg) {
    std::set<std::string> updated_agents(connected_agents_msg->agent_namespaces.begin(),connected_agents_msg->agent_namespaces.end());

    std::set<std::string> added_agents, removed_agents;

    std::set_difference(updated_agents.begin(), updated_agents.end(),
                        connected_agents_.begin(), connected_agents_.end(),
                        std::inserter(added_agents, added_agents.begin()));

    std::set_difference(connected_agents_.begin(), connected_agents_.end(),
                        updated_agents.begin(), updated_agents.end(), 
                        std::inserter(removed_agents, removed_agents.begin()));

    for(const auto& agent: added_agents) {
        add_agent(agent);
        RCLCPP_INFO(this->get_logger(), "Added cmd_vel publisher for agent: %s", agent.c_str());
    }

    for(const auto& agent: removed_agents) {
        remove_agent(agent);
        RCLCPP_INFO(this->get_logger(), "Removed cmd_vel publisher for agent: %s", agent.c_str());
    }

    connected_agents_ = updated_agents;
}

void PX4Teleop::add_agent(const std::string &agent_name) {
    std::string topic = "/" + agent_name + "/setpoint_velocity/cmd_vel_unfiltered";
    
    auto publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            topic,
            10
        );

    cmd_vel_publishers_.insert({agent_name, publisher});
    
    // if this is the first agent, point iterator to beginning of map
    if(cmd_vel_publishers_.size() == 1)
        agent_iterator_ = cmd_vel_publishers_.begin();
        setpoint_vel_.header.frame_id = agent_name;

}

void PX4Teleop::remove_agent(const std::string &agent_name) {
    // check if currently controlling this agent, if so, set iterator to beginning of map
    if(agent_name == agent_iterator_->first) {
        cmd_vel_publishers_.erase(agent_name);
        agent_iterator_ = cmd_vel_publishers_.begin();
    }
    else
        cmd_vel_publishers_.erase(agent_name);
}

bool PX4Teleop::initialize_origin_rotation() {
    this->declare_parameter("origin_r", 0.0);

    if (this->get_parameter("origin_r", origin_r_)) {
        RCLCPP_INFO(this->get_logger(), "Park origin rotation set to %.4f rad.", origin_r_);
        cos_origin_ = cos(origin_r_);
        sin_origin_ = sin(origin_r_);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Park origin rotation not set. Exiting.");
        return false;
    }
}