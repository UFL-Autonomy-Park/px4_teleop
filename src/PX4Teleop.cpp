#include "PX4Teleop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PX4Teleop::PX4Teleop() : Node("px4_teleop_node"),
							joy_handler_(this),
							gpos_init_(false),
							pose_init_(false),
							landing_requested_(false),
							sim_mode_(true),
							alt_init_(false),
							mission_takeoff_requested_(false),
							mission_land_requested_(false)
{

    px4_id_ = std::string(this->get_namespace()).substr(1);

    px4_safety.initialize(this);
	init_publishers();
	init_subscribers();
	init_service_clients();
    init_origin_rotation();
	

    RCLCPP_INFO(this->get_logger(), "PX4 Teleop Initialized.");
}

void PX4Teleop::init_publishers() {

	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(0), rmw_qos_profile_default);
	qos_profile.best_effort();
	qos_profile.durability_volatile();
	qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

	cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
	    "setpoint_velocity/cmd_vel", 
		10
        );

	pmr_pub_ = this->create_publisher<swarm_interfaces::msg::PrepareMissionResponse>(
		"/fleet_manager/prepare_mission/response",
		qos_profile
		);
		
	itr_pub_ = this->create_publisher<swarm_interfaces::msg::InitiateTakeoffResponse>(
		"/fleet_manager/initiate_takeoff/response",
		qos_profile
		);

	ilr_pub_ = this->create_publisher<swarm_interfaces::msg::InitiateLandResponse>(
		"/fleet_manager/initiate_land/response",
		qos_profile
		);
	
}

void PX4Teleop::init_subscribers() {
	
	rclcpp::QoS qos_connected_agents(rclcpp::KeepLast(10), rmw_qos_profile_default);
	qos_connected_agents.reliable();
    qos_connected_agents.transient_local(); // latched topics
	
	rclcpp::QoS qos_profile(rclcpp::KeepLast(10), rmw_qos_profile_default);
	qos_profile.best_effort();
	qos_profile.durability_volatile();

    connected_agents_sub_ = this->create_subscription<swarm_interfaces::msg::ConnectedAgents>(
		"/fleet_manager/connected_agents",
		qos_profile,
		std::bind(&PX4Teleop::connected_agents_callback, this, _1)
	);
    
	pmc_sub_ = this->create_subscription<swarm_interfaces::msg::PrepareMissionCommand>(
		"/fleet_manager/prepare_mission/cmd",
		qos_profile,
		std::bind(&PX4Teleop::pmc_callback, this, _1)
	);

	itc_sub_ = this->create_subscription<swarm_interfaces::msg::InitiateTakeoffCommand>(
		"/fleet_manager/initiate_takeoff/cmd",
		qos_profile,
		std::bind(&PX4Teleop::itc_callback, this, _1)
	);

	ilc_sub_ = this->create_subscription<swarm_interfaces::msg::InitiateLandCommand>(
		"/fleet_manager/initiate_land/cmd",
		qos_profile,
		std::bind(&PX4Teleop::ilc_callback, this, _1)
	);

	smc_sub_ = this->create_subscription<swarm_interfaces::msg::StartMissionCommand>(
		"/fleet_manager/start_mission/cmd",
		qos_profile,
		std::bind(&PX4Teleop::smc_callback, this, _1)
	);

	joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
		"/joy",
		qos_profile,
		std::bind(&PX4Teleop::joy_callback, this, _1)
	);

	state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
		"state",
		qos_profile,
		std::bind(&PX4Teleop::state_callback, this ,_1)
	);

	ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
		"extended_state",
		qos_profile,
		std::bind(&PX4Teleop::ext_state_callback, this, _1)
	);

    altitude_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>(
		"altitude",
		qos_profile,
		std::bind(&PX4Teleop::altitude_callback, this, _1)
	);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"autonomy_park/pose",
		qos_profile,
		[this](const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) 
		{
			apark_pose_ = pose_msg->pose;
			if (!pose_init_) pose_init_ = true;
		}
	);

	gpos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
		"global_position/global",
		qos_profile,
		[this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
		{
			global_pose_.position.latitude = msg->latitude;
			global_pose_.position.longitude = msg->longitude;
			global_pose_.position.altitude = msg->altitude;
			if(!gpos_init_) gpos_init_ = true;
		}
	);

	active_agent_sub_ = this->create_subscription<std_msgs::msg::String>(
		"/fleet_manager/active_agent",
		qos_profile,
		[this](const std_msgs::msg::String::SharedPtr active_agent)
		{
			active_agent_id_ = active_agent->data;
		}
	);
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

void PX4Teleop::control_input() {
	RCLCPP_INFO(this->get_logger(), "follower sending control input");

	// check leaders position
	geometry_msgs::msg::Pose target_pose = neighbor_poses_[leader_].pose;

	// offset by follower distance
	target_pose.position.x += follower_offset_[0];
	target_pose.position.y += follower_offset_[1];
	target_pose.position.z += follower_offset_[2];

	// P controller
	float vx = k_ * (target_pose.position.x - apark_pose_.position.x);
	float vy = k_ * (target_pose.position.y - apark_pose_.position.y);
	float vz = k_ * (target_pose.position.z - apark_pose_.position.z);

    // create velocity command
    geometry_msgs::msg::Twist unsafe_cmd_vel;
    unsafe_cmd_vel.linear.x = vx;
    unsafe_cmd_vel.linear.y = vy;
    unsafe_cmd_vel.linear.z = vz;
    unsafe_cmd_vel.angular.z = 0.0;

    // Generate safe velocity command
    geometry_msgs::msg::Twist safe_cmd_vel = px4_safety.compute_safe_cmd_vel(apark_pose_, unsafe_cmd_vel);

    // Convert safe autonomy park X/Y velocity command to ENU frame
	geometry_msgs::msg::TwistStamped vel_enu;
	vel_enu.header.frame_id = px4_id_;
	vel_enu.header.stamp = this->get_clock()->now();
	vel_enu.twist.linear.x = cos_origin_*safe_cmd_vel.linear.x + sin_origin_*safe_cmd_vel.linear.y;
	vel_enu.twist.linear.y = -sin_origin_*safe_cmd_vel.linear.x + cos_origin_*safe_cmd_vel.linear.y;
	vel_enu.twist.linear.z = safe_cmd_vel.linear.z;
	vel_enu.twist.angular.z = safe_cmd_vel.angular.z;

	// publish velocity command to px4
	cmd_vel_publisher_->publish(vel_enu);
}

void PX4Teleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (!pose_init_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring joy inputs until agent pose is initialized.");
        return;
    }
	else if (active_agent_id_ != px4_id_)
		return;

	else if (px4_id_ != leader_)
		return;

    // process joy message with joy handler (returns struct with actions)
    JoyHandler::joy_action action = joy_handler_.process(joy_msg);

	// handle arm/takeoff
	if (action.arm == true) {
		if (current_state_.armed && landed_state_ == on_ground) {
			send_tol_request(true);
		}
		else {
			send_arming_request(true);
		}
	}

	// handle disarm/land
	if (action.disarm == true) {
		if (landed_state_ == on_ground) {
			send_arming_request(false);
		}
		else {
			if (current_state_.mode == "AUTO.LOITER") {
				RCLCPP_INFO(this->get_logger(), "received command to send landing request");
				send_tol_request(false);
			}
			else {
				RCLCPP_INFO(this->get_logger(), "drone in air, sending loiter request before landing, mode: %s", current_state_.mode.c_str());
				landing_requested_ = true;

				auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
				request->custom_mode = "AUTO.LOITER";
				auto set_mode_request = set_mode_client_->async_send_request(
					request,
					std::bind(&PX4Teleop::loiter_mode_response_callback, this, _1)
				);
			}
		}
	}
	
	// handle offboard request
	if (action.offboard == true && current_state_.mode != "OFFBOARD") {
		auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		request->custom_mode = "OFFBOARD";
		auto set_mode_request = set_mode_client_->async_send_request(
			request,
			[this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
				auto response = future.get();

				if (response->mode_sent) {
					RCLCPP_INFO(this->get_logger(), "Offboard mode request succeeded.");
				} else {
					RCLCPP_ERROR(this->get_logger(), "Offboard mode request failed!");
				}
			}
		);
	}

	// handle loiter request (same button as offboard)
	else if (action.offboard == true && current_state_.mode == "OFFBOARD") {
		auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		request->custom_mode = "AUTO.LOITER";
		auto set_mode_request = set_mode_client_->async_send_request(
			request,
			std::bind(&PX4Teleop::loiter_mode_response_callback, this, _1)
		);
	}

    // handle command velocity
    geometry_msgs::msg::Twist unsafe_cmd_vel;
    unsafe_cmd_vel.linear.x = action.linear_x;
    unsafe_cmd_vel.linear.y = action.linear_y;
    unsafe_cmd_vel.linear.z = action.linear_z;
    unsafe_cmd_vel.angular.z = action.angular_z;

    //Generate safe velocity command
    geometry_msgs::msg::Twist safe_cmd_vel = px4_safety.compute_safe_cmd_vel(apark_pose_, unsafe_cmd_vel);

    //Convert safe autonomy park X/Y velocity command to ENU frame
	geometry_msgs::msg::TwistStamped vel_enu;
	vel_enu.header.frame_id = px4_id_;
	vel_enu.header.stamp = this->get_clock()->now();
	vel_enu.twist.linear.x = cos_origin_*safe_cmd_vel.linear.x + sin_origin_*safe_cmd_vel.linear.y;
	vel_enu.twist.linear.y = -sin_origin_*safe_cmd_vel.linear.x + cos_origin_*safe_cmd_vel.linear.y;
	vel_enu.twist.linear.z = safe_cmd_vel.linear.z;
	vel_enu.twist.angular.z = safe_cmd_vel.angular.z;

	// publish velocity command to px4
	cmd_vel_publisher_->publish(vel_enu);
}


void PX4Teleop::connected_agents_callback(const swarm_interfaces::msg::ConnectedAgents::SharedPtr connected_agents_msg) {
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

		if (agent != px4_id_)
			RCLCPP_INFO(this->get_logger(), "Added neighbor: %s", agent.c_str());
    }

    for(const auto& agent: removed_agents) {
        remove_agent(agent);
        RCLCPP_INFO(this->get_logger(), "Removed neighbor: %s", agent.c_str());
    }

    connected_agents_ = updated_agents;
}

void PX4Teleop::add_agent(const std::string &agent_name) {

	rclcpp::QoS qos_profile(rclcpp::KeepLast(10), rmw_qos_profile_default);
	qos_profile.best_effort();
	qos_profile.durability_volatile();

    std::string agent_namespace = "/" + agent_name;
	
	auto neighbor_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		agent_namespace + "/autonomy_park/pose",
		qos_profile,
		[this, agent_namespace, agent_name](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

			neighbor_poses_.insert_or_assign(agent_name, *msg);
		}
	);

	auto neighbor_velocity_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
		agent_namespace + "/setpoint_velocity/cmd_vel",
		qos_profile,
		[this, agent_namespace, agent_name](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {

			neighbor_velocities_.insert_or_assign(agent_name, *msg);
		}
	);

	auto neighbor_state_sub = this->create_subscription<mavros_msgs::msg::State>(
		agent_namespace + "/state",
		qos_profile,
		[this, agent_namespace, agent_name](const mavros_msgs::msg::State::SharedPtr msg) {

			neighbor_states_.insert_or_assign(agent_name, *msg);
		}
	);

	auto neighbor_ext_state_sub = this->create_subscription<mavros_msgs::msg::ExtendedState>(
		agent_namespace + "/extended_state",
		qos_profile,
		[this, agent_namespace, agent_name](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {

			neighbor_ext_states_.insert_or_assign(agent_name, *msg);
		}
	);

	neighbor_pose_subscriptions_.insert({agent_name, neighbor_pose_sub});
	neighbor_velocity_subscriptions_.insert({agent_name, neighbor_velocity_sub});
	neighbor_state_subscriptions_.insert({agent_name, neighbor_state_sub});	
	neighbor_ext_state_subscriptions_.insert({agent_name, neighbor_ext_state_sub});
}

void PX4Teleop::remove_agent(const std::string &agent_name) {
	neighbor_pose_subscriptions_.erase(agent_name);
	neighbor_velocity_subscriptions_.erase(agent_name);
	neighbor_state_subscriptions_.erase(agent_name);
	neighbor_ext_state_subscriptions_.erase(agent_name);
	neighbor_poses_.erase(agent_name);
	neighbor_velocities_.erase(agent_name);
	neighbor_states_.erase(agent_name);
	neighbor_ext_states_.erase(agent_name);
	connected_agents_.erase(agent_name);
}

void PX4Teleop::state_callback(const mavros_msgs::msg::State::SharedPtr state_msg) {

    if (state_msg->armed && !current_state_.armed) {
        RCLCPP_WARN(this->get_logger(), "Armed");
    }
	else if (!state_msg->armed && current_state_.armed) {
        RCLCPP_WARN(this->get_logger(), "Disarmed");
    }

    if (state_msg->mode == "AUTO.LOITER" && current_state_.mode != "AUTO.LOITER") {
        RCLCPP_WARN(this->get_logger(), "Loiter mode enabled.");
        
        if (landing_requested_) {
            //Send landing request
            send_tol_request(false);
            landing_requested_ = false;
        }
    } 
	else if (state_msg->mode == "OFFBOARD" && current_state_.mode != "OFFBOARD") {

		// if on ground when offboard is enabled, disable it
		if (landed_state_ == on_ground) {
			auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
			request->custom_mode = "AUTO.LOITER";
			auto set_mode_result = set_mode_client_->async_send_request(request, std::bind(&PX4Teleop::loiter_mode_response_callback, this, _1));
		}

		else {
			RCLCPP_WARN(this->get_logger(), "Offboard mode enabled.");
		}
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

	geographic_msgs::msg::GeoPose takeoff_pose = global_pose_; 

    if (takeoff) {
        RCLCPP_WARN(this->get_logger(), "Sending takeoff request with altitude %.2f, takeoff_height: %.2f", altitude_amsl_, takeoff_height_);
        
        tol_request->yaw = quat_to_yaw(takeoff_pose.orientation);
        tol_request->latitude = takeoff_pose.position.latitude;
        tol_request->longitude = takeoff_pose.position.longitude;
        tol_request->altitude = altitude_amsl_ - apark_pose_.position.z + takeoff_height_;

        auto tol_result = takeoff_client_->async_send_request(tol_request, std::bind(&PX4Teleop::tol_response_callback, this, _1));
    } else {
        RCLCPP_WARN(this->get_logger(), "Sending landing request.");

        //Set landing pos to current @ 0 meter altitude
        tol_request->yaw = quat_to_yaw(takeoff_pose.orientation);
        tol_request->latitude = takeoff_pose.position.latitude;
        tol_request->longitude = takeoff_pose.position.longitude;
        tol_request->altitude = 0.0;

        auto tol_result = land_client_->async_send_request(tol_request, std::bind(&PX4Teleop::tol_response_callback, this, _1));
    }
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


void PX4Teleop::pmc_callback(swarm_interfaces::msg::PrepareMissionCommand::SharedPtr pmc_msg) {

	mission_id_ = pmc_msg->mission_id;

	RCLCPP_INFO(this->get_logger(), "received prepare mission request. Mission ID: %s.", mission_id_.c_str());

	// load mission parameters, do pre-flight checks
	if (mission_id_ == "Formation Hold") {
		this->declare_parameter("leader", "");
		this->declare_parameter("followers", std::vector<std::string>{});
		this->declare_parameter("follower_offset", std::vector<double>{});
		this->get_parameter("leader", leader_);
		this->get_parameter("followers", followers_);
		
		if (px4_id_ != leader_) {
			std::string param_name = px4_id_ + ".follower_offset";
			this->get_parameter(param_name, follower_offset_);

			RCLCPP_INFO(this->get_logger(), "Agent offset: %.2f, %.2f, %.2f",
						follower_offset_[0], follower_offset_[1], follower_offset_[2]);
		}
	}
	
	// for testing, just directly sending back a true response
	swarm_interfaces::msg::PrepareMissionResponse pmr_msg;
	pmr_msg.agent_id = px4_id_;
	pmr_msg.ready = true;
	pmr_msg.message = std::string("testing");
	pmr_pub_->publish(pmr_msg);

}

void PX4Teleop::ilc_callback(swarm_interfaces::msg::InitiateLandCommand::SharedPtr ilc_msg) {
	land_height_ = ilc_msg->land_height;

	bool valid_spacing = true;
	for (auto agent : neighbor_poses_) {
		if (compute_horizontal_separation(apark_pose_.position, agent.second.pose.position) < minimum_takeoff_separation_)
			valid_spacing = false;
	}
	
	if (valid_spacing == true) {
		mission_land_requested_ = true;
		send_tol_request(false);
	}

}
void PX4Teleop::itc_callback(swarm_interfaces::msg::InitiateTakeoffCommand::SharedPtr itc_msg) {

	RCLCPP_INFO(this->get_logger(), "received initiate takeoff request with height: %.2f.", itc_msg->takeoff_height);

	takeoff_height_ = itc_msg->takeoff_height;

	// 1. valid takeoff location
	bool position_lock = false;
	if(pose_init_ && gpos_init_ && alt_init_) {
		position_lock = true;
	}
	
	// 2. valid agent spacing
	bool valid_spacing = true;
	for (auto agent : neighbor_poses_) {

		float dist = compute_horizontal_separation(apark_pose_.position, agent.second.pose.position) < minimum_takeoff_separation_;

		if (compute_horizontal_separation(apark_pose_.position, agent.second.pose.position) < minimum_takeoff_separation_) {
			RCLCPP_INFO(this->get_logger(), "Invalid spacing with agent %s: %.2f", agent.first.c_str(), dist);
			valid_spacing = false;
		}

		else {
			RCLCPP_INFO(this->get_logger(), "Safe distance from agent %s: %.2f", agent.first.c_str(), dist);
		}
	}

	if (position_lock && valid_spacing) {
		mission_takeoff_requested_ = true;
		send_arming_request(true);
	}


}

void PX4Teleop::smc_callback(swarm_interfaces::msg::StartMissionCommand::SharedPtr smc_msg) {

	RCLCPP_INFO(this->get_logger(), "received start mission request.");
	
	if (mission_id_ == "Formation Hold") {
		control_input_timer_ = this->create_wall_timer(std::chrono::duration<double>(0.02), 
												 std::bind(&PX4Teleop::control_input, this));

		this->declare_parameter("k-gain", 0.0);
		this->get_parameter("k-gain", k_);
	}

	mission_start_time_ = smc_msg->timestamp;
}

void PX4Teleop::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) {
    //Gazebo sim uses monotonic altitude, physical drone uses local tied to bottom_clearance via lidar
    if (sim_mode_) {
        apark_pose_.position.z = msg->local;
    } else {
        apark_pose_.position.z = msg->local;
    }
    
    altitude_amsl_ = msg->amsl;

    //Set initialization flag
    if (!alt_init_) alt_init_ = true;
}

void PX4Teleop::tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {

		if (mission_takeoff_requested_) {

			RCLCPP_INFO(this->get_logger(), "Mission TOL request succeeded. Result=%d", response->result);

			swarm_interfaces::msg::InitiateTakeoffResponse itr_msg;
			itr_msg.agent_id = px4_id_;
			itr_msg.success = true;
			itr_msg.message = "testing";
			itr_pub_->publish(itr_msg);

			mission_takeoff_requested_ = false;
		}

		else if (mission_land_requested_) {
			
			swarm_interfaces::msg::InitiateLandResponse ilr_msg;
			ilr_msg.agent_id = px4_id_;
			ilr_msg.success = true;
			ilr_msg.message = "testing";
			ilr_pub_->publish(ilr_msg);

			mission_land_requested_ = false;

		}

		else {
			RCLCPP_INFO(this->get_logger(), "TOL request succeeded. Result=%d", response->result);
		}
    } else {
        RCLCPP_ERROR(this->get_logger(), "TOL request failed!");
    }
}

void PX4Teleop::send_arming_request(bool arm) {
    bool valid_request = false;
    if (arm) {
        if (!current_state_.armed) {
            RCLCPP_WARN(this->get_logger(), "Sending arm request.");
            valid_request = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Device already armed - arm request ignored.");
            return;
        }
    } else {
        if (current_state_.armed) {
            RCLCPP_WARN(this->get_logger(), "Sending disarm request.");
            valid_request = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Device already disarmed - disarm request ignored.");
            return;
        }
    }

    if (valid_request) {
        auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_request->value = arm;
        auto arm_result = arm_client_->async_send_request(arm_request, std::bind(&PX4Teleop::arm_response_callback, this, _1));
    }
}

void PX4Teleop::arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto response = future.get();


	if (response->success) {

		if (mission_takeoff_requested_) {

			RCLCPP_INFO(this->get_logger(), "Mission Arm request succeeded. Result=%d", response->result);
			std::this_thread::sleep_for(2000ms);
			send_tol_request(true);
		}
		
		else {
			RCLCPP_INFO(this->get_logger(), "Arm/disarm request succeeded. Result=%d", response->result);
		}
    } 

	else {
        RCLCPP_ERROR(this->get_logger(), "Arm/disarm request failed!");
    }
}
