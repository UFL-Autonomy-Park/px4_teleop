#include "PX4Teleop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PX4Teleop::PX4Teleop() : Node("px4_teleop_node"), pose_init_(false), joy_handler_(shared_from_this()) {
	RCLCPP_INFO(this->get_logger(), "Initializing PX4 Teleop Node");

    //joy_handler_ = JoyHandler(this);

    //Get my namespace (remove the slash with substr)
    agent_id_ = std::string(this->get_namespace()).substr(1);

    //Get controller parameters
    this->declare_parameter("x_axis", -1);
    this->declare_parameter("y_axis", -1);
    this->declare_parameter("z_axis", -1);
    this->declare_parameter("yaw_axis", -1);

    this->declare_parameter("x_vel_max", 1.0);
    this->declare_parameter("y_vel_max", 1.0);
    this->declare_parameter("z_vel_max", 1.0);
    this->declare_parameter("yaw_vel_max", 1.0);

    this->get_parameter("x_axis", axes_.x.axis);
    this->get_parameter("y_axis", axes_.y.axis);
    this->get_parameter("z_axis", axes_.z.axis);
    this->get_parameter("yaw_axis", axes_.yaw.axis);

    this->get_parameter("x_vel_max", axes_.x.factor);
    this->get_parameter("y_vel_max", axes_.y.factor);
    this->get_parameter("z_vel_max", axes_.z.factor);
    this->get_parameter("yaw_vel_max", axes_.yaw.factor);

    //RCLCPP_INFO(this->get_logger(), "Loaded controller axis parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d", axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis);

    //Get park rotation from params (not optimal but needed because vel commands go to global ENU frame)
    this->declare_parameter("origin_r", 0.0);
    if (this->get_parameter("origin_r", origin_r_)) {
        RCLCPP_INFO(this->get_logger(), "Park origin rotation set to %.4f rad.", origin_r_);
        cos_origin_ = cos(origin_r_);
        sin_origin_ = sin(origin_r_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Park origin rotation not set. Exiting.");
        rclcpp::shutdown();
        return;
    }

    px4_safety.initialize(this);

    setpoint_vel_.header.frame_id = agent_id_;

    //Publish setpoints at 50Hz
    setpoint_timer_ = this->create_wall_timer(20ms, std::bind(&PX4Teleop::publish_setpoint, this));
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PX4Teleop::joy_callback, this, _1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("autonomy_park/pose", 10, std::bind(&PX4Teleop::pose_callback, this, _1));

    // setup qos profile
    qos_profile_.reliable();
    qos_profile_.transient_local();
    connected_agents_sub_ = this->create_subscription<fleet_manager::msg::ConnectedAgents>("/connected_agents", qos_profile_, std::bind(&PX4Teleop::connected_agents_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "PX4 Teleop Initialized.");
}

void PX4Teleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    // call processing function in JoyHandler
    // create cmd vel message from returned actions
    // check for TOL and switch agent
    // call publish vel function
    
    // check for switch agent button press (R1)
    if(joy_msg->buttons[5] > 0 && !switch_agent_state_ && !cmd_vel_publishers_.empty()) {
        switch_agent_state_ = true;
        agent_iterator_++;

        if(agent_iterator_ == cmd_vel_publishers_.end())
            agent_iterator_ = cmd_vel_publishers_.begin();

        RCLCPP_INFO(this->get_logger(), "controlling agent: %s", agent_iterator_->first);
    }
    else if(joy_msg->buttons[5] == 0 && switch_agent_state_)
        switch_agent_state_ = false;


    if (!pose_init_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring joy inputs until agent pose is initialized.");
        return;
    }

    double apark_vx = get_axis(joy_msg, axes_.x);
    double apark_vy = get_axis(joy_msg, axes_.y);

    geometry_msgs::msg::Twist unsafe_cmd_vel;
    unsafe_cmd_vel.linear.x = apark_vx;
    unsafe_cmd_vel.linear.y = apark_vy;
    unsafe_cmd_vel.linear.z = get_axis(joy_msg, axes_.z);
    unsafe_cmd_vel.angular.z = get_axis(joy_msg, axes_.yaw);

    //Generate safe velocity command
    geometry_msgs::msg::Twist safe_cmd_vel = px4_safety.compute_safe_cmd_vel(agent_pose_, unsafe_cmd_vel);

    //Convert autonomy park X/Y velocity command to ENU frame
    setpoint_vel_.twist.linear.x = cos_origin_*safe_cmd_vel.linear.x + sin_origin_*safe_cmd_vel.linear.y;
    setpoint_vel_.twist.linear.y = -sin_origin_*safe_cmd_vel.linear.x + cos_origin_*safe_cmd_vel.linear.y;
    setpoint_vel_.twist.linear.z = safe_cmd_vel.linear.z;
    setpoint_vel_.twist.angular.z = safe_cmd_vel.angular.z;

    // RCLCPP_WARN(this->get_logger(), "Orig Twist: (%.4f, %.4f, %.4f)", unsafe_cmd_vel.linear.x, unsafe_cmd_vel.linear.y, unsafe_cmd_vel.linear.z);
    // RCLCPP_WARN(this->get_logger(), "Safe Twist: (%.4f, %.4f, %.4f)", safe_cmd_vel.linear.x, safe_cmd_vel.linear.y, safe_cmd_vel.linear.z);
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

double PX4Teleop::get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis) {
    if (axis.axis < 0 || std::abs(axis.axis) > (int)joy_msg->axes.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Axis %d out of range, joy has %d axes", axis.axis, (int)joy_msg->axes.size());
        return -1;
    }

    double output = joy_msg->axes[std::abs(axis.axis)] * axis.factor + axis.offset;

    return output;
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
    std::string topic = "/" + agent_name + "/setpoint_velocity/cmd_vel";
    
    auto publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            topic,
            10
        );

    cmd_vel_publishers_.insert({agent_name, publisher});
    
    // if this is the first agent, point iterator to beginning of map
    if(cmd_vel_publishers_.size() == 1)
        agent_iterator_ = cmd_vel_publishers_.begin();
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
