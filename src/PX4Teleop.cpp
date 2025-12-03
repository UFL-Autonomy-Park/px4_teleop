#include "PX4Teleop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PX4Teleop::PX4Teleop() : Node("px4_teleop_node"),
                          joy_handler_(this),
                          pose_init_(false),
                          landing_requested(false)
{

    px4_id_ = std::string(this->get_namespace()).substr(1);

    px4_safety.initialize(this);
	init_publishers();
	init_subscribers();
	init_service_clients();
    init_origin_rotation();

    RCLCPP_INFO(this->get_logger(), "PX4 Teleop Initialized.");
}

void PX4Teleop::init_publishers() [
	cmd_vel_publsher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            topic,
            10
        );
}

void PX4Teleop::init_subscribers() {
	
    // setup qos profile and subscribers
	rclcpp::QoS qos_connected_agents(rclcpp::KeeyLast(10), rmw_qos_profile_default);
	qos_connected_agents.reliable();
    qos_connected_agents.transient_local(); // latched topics
    connected_agents_sub_ = this->create_subscription<fleet_manager::msg::ConnectedAgents>("/connected_agents", qos_profile_, std::bind(&PX4Teleop::connected_agents_callback, this, _1));
    
	rclcpp::QoS qos_profile(rclcpp::KeepLast(10), rmw_qos_profile_defaults);
	qos_profile.best_effort();
	qos_profile.durability_volatile();

	joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", qos_profile, std::bind(&PX4Teleop::joy_callback, this, _1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("autonomy_park/pose", qos_profile, std::bind(&PX4Teleop::pose_callback, this, _1));
	gpos_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>("autonomy_park/global_pose", qos_profile, std::bind(&PX4Teleop::apark_global_pose_callback, this, _1));
	ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("extended_state", qos_profile, std::bind(&PX4Teleop::ext_state_callback, this, _1));
    altitude_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>("altitude", qos_profile, std::bind(&PX4Teleop::altitude_callback, this, _1));
	active_agent_sub_ = this->create_subscription<std_msgs::msg::String>("/fleet_manager/active_agent", qos_profile, std::bind&(PX4Teleop::active_agent_callback, this, _1));
}

void PX4Teleop::init_service_clients() {

    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("set_mode"); 
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("cmd/takeoff");
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("cmd/land");
	arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("cmd/arming");

    //Wait for set mode service
    while (!set_mode_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for mode service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Mode service not available, waiting again...");
    }

    //Wait for arm service
    while (!arm_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for arming service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Arming service not available, waiting again...");
    }

    //Wait for TOL service
    while (!takeoff_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for TOL service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "TOL service not available, waiting again...");
    }
}

void PX4Teleop::init_origin_rotation() {
    this->declare_parameter("origin_r", 0.0);

    if (this->get_parameter("origin_r", origin_r_)) {
        RCLCPP_INFO(this->get_logger(), "Park origin rotation set to %.4f rad.", origin_r_);
        cos_origin_ = cos(origin_r_);
        sin_origin_ = sin(origin_r_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Park origin rotation not set. Exiting.");
        rclcpp::shutdown();
    }
}

void PX4Teleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (!pose_init_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring joy inputs until agent pose is initialized.");
        return;
    }
	else if (active_agent_id_ != px4_id_)
		return;

    // process joy message with joy handler (returns struct with actions)
    JoyHandler::joy_action action = joy_handler_.process(joy_msg);

    // handle mode switching
	
	// handle takeoff
	if(action.takeoff == true && landed_state_ == on_ground) {
		send_tol_request(true);

	// handle land
	if(action.land == true && current_state_.armed) {
		RCLCPP_INFO(this->get_logger(), "Request to land sent.");

		if(current_state_->mode == "AUTO.LOITER"){ 
			senc_tol_request(false);
		}
		else {
			landing_requested_ = true;

			auto request = std::make_shared<mavros_msgs::msg::SetMode::Request();
			request->custom_mode = "AUTO.LOITER";
			auto set_mode_request = set_mode_client->async_send_request(request, std::bind&(PX4Teleop::loiter_mode_response_callback, this, _1));
	}
    // handle command velocity
    
    // check for switch agent action
    if(action.switch_agent == true && !cmd_vel_publishers_.empty()) {
        switch_agent();
    }

    // store unfiltered velocity command
    geometry_msgs::msg::Twist unsafe_cmd_vel;
    unsafe_cmd_vel.linear.x = action.linear_x;
    unsafe_cmd_vel.linear.y = action.linear_y;
    unsafe_cmd_vel.linear.z = action.linear_z;
    unsafe_cmd_vel.angular.z = action.angular_z;

    //Generate safe velocity command
    geometry_msgs::msg::Twist safe_cmd_vel = px4_safety.compute_safe_cmd_vel(agent_pose_, unsafe_cmd_vel);

    //Convert safe autonomy park X/Y velocity command to ENU frame
    if(!cmd_vel_publishers_.empty()) {
        geometry_msgs::msg::TwistStamped vel_enu;
        vel_enu.header.frame_id = agent_iterator_->first;
        vel_enu.header.stamp = this->get_clock()->now();
        vel_enu.twist.linear.x = cos_origin_*safe_cmd_vel.linear.x + sin_origin_*safe_cmd_vel.linear.y;
        vel_enu.twist.linear.y = -sin_origin_*safe_cmd_vel.linear.x + cos_origin_*safe_cmd_vel.linear.y;
        vel_enu.twist.linear.z = safe_cmd_vel.linear.z;
        vel_enu.twist.angular.z = safe_cmd_vel.angular.z;

        // publish velocity command to px4
        agent_iterator_->second->publish(vel_enu);
    }
}

void PX4Teleop::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    //Update PX4 agent pose
    agent_pose_ = pose_msg->pose;

    if (!pose_init_) pose_init_ = true;
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
}

void PX4Teleop::remove_agent(const std::string &agent_name) {
    // check if currently controlling this agent, if so, set iterator to beginning of map
    auto it = cmd_vel_publishers_.find(agent_name);
   if(it == cmd_vel_publishers_.end()) return;
    
    if(it == agent_iterator_) {
        agent_iterator_ = cmd_vel_publishers_.erase(it);
        if(agent_iterator_ == cmd_vel_publishers_.end() && !cmd_vel_publishers_.empty())
            agent_iterator_ = cmd_vel_publishers_.begin();
    }
    else
        cmd_vel_publishers_.erase(it);
}

void PX4Teleop::switch_agent() {
    agent_iterator_++;

    if(agent_iterator_ == cmd_vel_publishers_.end())
        agent_iterator_ = cmd_vel_publishers_.begin();

    RCLCPP_INFO(this->get_logger(), "controlling agent: %s", agent_iterator_->first.c_str());
}

void PX4Teleop::state_callback(const mavros_msgs::msg::State::SharedPtr state_msg) {

    if (state_msg->armed && !current_state_.armed) {
        RCLCPP_WARN(this->get_logger(), "Armed");
    }
	else if (!state_msg->armed && current_state_.armed) {
        RCLCPP_WARN(this->get_logger(), "Disarmed");
    }
    if (state_msg->mode == "AUTO.LOITER" && current_state_.mode != loiter_str_) {
        RCLCPP_WARN(this->get_logger(), "Loiter mode enabled.");
        
        if (landing_requested_) {
            //Send landing request
            send_tol_request(false);
            landing_requested_ = false;
        }
    } else if (state_msg->mode == "OFFBOARD" && current_state_.mode != offboard_str_) {
        RCLCPP_WARN(this->get_logger(), "Offboard mode enabled.");
    }

    current_state_ = *state_msg;
}

double PX4Teleop::quat_to_yaw(geometry_msgs::msg::Quaternion quat) {

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

void PX4Teleop::send_tol_request(bool takeoff) {
    auto tol_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    if (takeoff) {
        RCLCPP_WARN(this->get_logger(), "Sending takeoff request.");
        
		geographic_msgs::msg::GeoPose takeoff_pose = apark_global_pose_; 
        tol_request->yaw = quat_to_yaw(takeoff_pose.orientation);
        tol_request->latitude = takeoff_pose.position.latitude;
        tol_request->longitude = takeoff_pose.position.longitude;
        tol_request->altitude = takeoff_pose.position.altitude + 2.0; // altitude in amsl

        auto tol_result = takeoff_client_->async_send_request(tol_request, std::bind(&PX4Telemetry::tol_response_callback, this, _1));
    } else {
        RCLCPP_WARN(this->get_logger(), "Sending landing request.");

        //Set landing pos to current @ 0 meter altitude
        tol_request->yaw = quat_to_yaw(takeoff_pose.orientation);
        tol_request->latitude = takeoff_pose.position.latitude;
        tol_request->longitude = takeoff_pose.position.longitude;
        tol_request->altitude = 0.0;

        auto tol_result = land_client_->async_send_request(tol_request, std::bind(&PX4Telemetry::tol_response_callback, this, _1));
    }
}

void PX4Teleop::apark_global_pose_callback(geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
	apark_global_pose_ = msg->pose;
}


void PX4Teleop::ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg) {

    if ((LandedState)ext_state_msg->landed_state != landed_state_) {
        switch (ext_state_msg->landed_state) {
            case undefined: {
                RCLCPP_ERROR(this->get_logger(), "Undefined landed state!");
                break;
            } case on_ground: {
                RCLCPP_WARN(this->get_logger(), "Entered on ground state.");
                break;
            } case in_air: {
                RCLCPP_WARN(this->get_logger(), "Entered in air state.");
                break;
            } case takeoff: {
                RCLCPP_WARN(this->get_logger(), "Entered takeoff state.");
                break;
            } case landing: {
                RCLCPP_WARN(this->get_logger(), "Entered landing state.");
                break;
            }
        }

        landed_state_ = (LandedState)ext_state_msg->landed_state;
    }
}

void PX4Teleop::loiter_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    auto response = future.get();

    if (response->mode_sent) {
        RCLCPP_INFO(this->get_logger(), "Loiter mode request succeeded.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Loiter mode request failed!");
    }
}

void PX4Teleop::active_agent_callback(const std_msgs::msg::String::SharedPtr active_agent)
{
	active_agent_id_ = *active_agent
}









