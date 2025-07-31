#include "ubmkz_interface.hpp"
#include <iostream>
#include <fstream>

namespace ds_dbw_can {
    UbmkzInterface::UbmkzInterface()
    :rclcpp::Node("ubmkz_interface")
{
    /* Set Up Parameters*/
    // TODO: Replace the parameter's so that they are loaded from ros_params
    base_frame_id_ = declare_parameter("base_frame_id", "base_link");   //Frame ID
    command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
    loop_rate_ = declare_parameter("loop_rate", 50.0);
    wheel_base_ = 2.98;
    steering_ratio_ = 14.6;

    /* subscriber */
    using std::placeholders::_1;
    using std::placeholders::_2;

    //from DbwNode
    sub_brake_ = create_subscription<ds_dbw_msgs::msg::BrakeReport>("/vehicle/brake_report", rclcpp::QoS{2}, 
        std::bind(&UbmkzInterface::callbackBrakeRpt, this, _1));
    //sub_throttle_ = create_subscription<ds_dbw_msgs::msg::ThrottleReport>("/vehicle/throttle_report", 2);
    sub_steering_ = std::make_unique<message_filters::Subscriber<ds_dbw_msgs::msg::SteeringReport>>(
        this, "/vehicle/steering_report");
    sub_gear_ = std::make_unique<message_filters::Subscriber<ds_dbw_msgs::msg::GearReport>>(
        this, "/vehicle/gear_report");
    sub_misc_1_ = std::make_unique<message_filters::Subscriber<ds_dbw_msgs::msg::Misc1Report>>(
        this, "/vehicle/misc_1_report");


    //from Autoware
    sub_control_cmd_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 1, std::bind(&UbmkzInterface::callbackControlCmd, this, _1));
    sub_control_cmd_ = std::make_unique<message_filters::Subscriber<Autoware_auto_control_msgs::msg::AckermannControlCommand>>(
        this, "/control/command/control_cmd");
    sub_gear_cmd_ = create_subscription<GearCommand>("input/gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_gear_cmd_ = *msg; });
    //sub_manual_gear_cmd_ = create_subscription<GearCommand>("input/manual_gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_manual_gear_cmd_ = *msg; });    
    sub_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>("input/turn_indicators_command", QoS{1},std::bind(&SimplePlanningSimulator::on_turn_indicators_cmd, this, _1));
    sub_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>("input/hazard_lights_command", QoS{1},std::bind(&SimplePlanningSimulator::on_hazard_lights_cmd, this, _1));
    sub_emergency_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
        "/control/command/emergency_cmd", 1, std::bind(&UbmkzInterface::callbackEmergencyCmd, this, _1));
    control_mode_server_ = create_service<ControlModeCommand>(
        "input/control_mode_request", std::bind(&UbmkzInterface::onControlModeRequest, this, _1, _2));
    
    
    //from UlcNode
    sub_ulc_rpt_ = std::make_unique<message_filters::Subscriber<dataspeed_ulc_msgs::msg::UlcReport>>(
        this, "/vehicle/ulc_report");

    //from DbwNode
    sub_enable_ = create_subscription<std_msgs::msg::Bool>("/vehicle/dbw_enabled", rclcpp::QoS(2).transient_local(),
        std::bind(&UbmkzInterface::recvDbwEnabled, this, _1));   

    //synchronizer
    ubmkz_feedbacks_sync_ =
        std::make_unique<message_filters::Synchronizer<UbmkzFeedbacksSyncPolicy>>(
        UbmkzFeedbacksSyncPolicy(10), *sub_steering_, *sub_gear_, *sub_misc_1_, *sub_ulc_rpt_);
    
    ubmkz_feedbacks_sync_->registerCallback(std::bind(
        &UbmkzInterface::callbackInterface, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));
    
     /* publisher */
    //to UlcNode
    pub_ulc_cmd_=create_publisher<dataspeed_ulc_msgs::msg::UlcCmd>("/vehicle/ulc_cmd", 2);

    //to Autoware
    pub_control_mode_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS{1});
    pub_vehicle_twist_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS{1});
    pub_steering_status_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS{1});
    pub_gear_status_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_status", rclcpp::QoS{1});
    pub_turn_indicators_status_ =
        create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
        "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
    pub_hazard_lights_status_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
        "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
    /*   pub_actuation_status_ =
        create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1); */
    pub_steering_wheel_status_ =
        create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);
    pub_door_status_ =
        create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);


    // Timer
    const auto period_ns = rclcpp::Rate(loop_rate_).period();
    timer_ = rclcpp::create_timer(
        this, get_clock(), period_ns, std::bind(&UbmkzInterface::publishCommands, this));        
}


void UbmkzInterface::callbackInterface(
    const ds_dbw_msgs::msg::SteeringReport::ConstSharedPtr steering_rpt,
    const ds_dbw_msgs::msg::GearReport::ConstSharedPtr gear_rpt,
    const ds_dbw_msgs::msg::Misc1Report::ConstSharedPtr misc1_rpt,
    const dataspeed_ulc_msgs::msg::UlcReport::ConstSharedPtr ulc_rpt)
{
    std_msgs::msg::Header header;
    header.frame_id = base_frame_id_;
    header.stamp = get_clock()->now();

    is_dbw_rpt_received_ = true;
    sub_steering_ptr_ = steering_rpt;
    sub_gear_ptr_ = gear_rpt;
    sub_misc1_ptr_ = misc1_rpt;
    sub_ulc_rpt_ptr_ = ulc_rpt;

    const double current_velocity = sub_steering_ptr_->speed;
    const double current_steer_wheel = sub_steering_ptr_->steering_wheel_angle;  // current vehicle steering wheel angle [rad]
    const double current_steer = current_steer_wheel / steering_ratio_;

    /* publish steering wheel status */
    {
        SteeringWheelStatusStamped steering_wheel_status_msg;
        steering_wheel_status_msg.stamp = header.stamp;
        //std::cout << "current_steer_wheel: " << current_steer_wheel << "rad" << std::endl;

        steering_wheel_status_msg.data = current_steer_wheel;
        pub_steering_wheel_status_->publish(steering_wheel_status_msg); 
  
    }

    /* publish current steering status */
    {
        autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
        steer_msg.stamp = header.stamp;
        steer_msg.steering_tire_angle = current_steer;
        pub_steering_status_->publish(steer_msg);
    }

    /*publish Velocity Report*/
    {
        autoware_auto_vehicle_msgs::msg::VelocityReport twist;
        twist.header = header;
        twist.longitudinal_velocity = current_velocity;                                 // [m/s]
        twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s] yaw rate

        pub_vehicle_twist_->publish(twist);
    }

    
    /* publish vehicle status control_mode */
    {
        autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
        control_mode_msg.stamp = header.stamp;

        if (dbw_enable_) {
        control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        } else {
        control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
        }
        pub_control_mode_->publish(control_mode_msg);
    }

    /* publish current shift */
    {
        autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
        gear_report_msg.stamp = header.stamp;
        const auto opt_gear_report = toAutowareShiftReport(*sub_gear_ptr_);
        if (opt_gear_report) {
        gear_report_msg.report = opt_gear_report;
        pub_gear_status_->publish(gear_report_msg);
        }
    }

    /*publish TurnIndicatorsReport*/
    {
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
        turn_msg.stamp = header.stamp;
        turn_msg.report = toAutowareTurnIndicatorsReport(*sub_misc1_ptr_);
        pub_turn_indicators_status_->publish(turn_msg);

        autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
        hazard_msg.stamp = header.stamp;
        hazard_msg.report = toAutowareHazardLightsReport(*sub_misc1_ptr_);
        pub_hazard_lights_status_->publish(hazard_msg);
    }

}


int32_t UbmkzInterface::toAutowareShiftReport(
  const ds_dbw_msgs::msg::GearReport & gear_rpt)
{
  using autoware_auto_vehicle_msgs::msg::GearReport;
  using ds_dbw_msgs::msg::Gear;

  if (gear_rpt.state.gear == ds_dbw_msgs::msg::Gear::PARK) {
    return static_cast<int32_t>(GearReport::PARK);
  }
  if (gear_rpt.state.gear == ds_dbw_msgs::msg::Gear::REVERSE){
    return static_cast<int32_t>(GearReport::REVERSE);
  }
  if (gear_rpt.state.gear == ds_dbw_msgs::msg::Gear::NEUTRAL){
    return static_cast<int32_t>(GearReport::NEUTRAL);
  }
  if (gear_rpt.state.gear == ds_dbw_msgs::msg::Gear::DRIVE){
    return static_cast<int32_t>(GearReport::DRIVE);
  }
  if (gear_rpt.state.gear==ds_dbw_msgs::msg::Gear::LOW){
    return static_cast<int32_t>(GearReport::LOW);
  }
  return static_cast<int32_t>(GearReport::NONE);
}

void UbmkzInterface::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand& msg)
{
  control_command_received_time_ = this->now();
  ulc_cmd_.header.frame_id = base_frame_id_;
  ulc_cmd_.header.stamp = get_clock()->now();
  // Populate command fields
  ulc_cmd_.pedals_mode = dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE;
  ulc_cmd_.linear_velocity = msg.longitudinal.speed;

  ulc_cmd_.yaw_command = sub_steering_ptr_->speed* tan(msg.lateral.steering_tire_angle)/wheel_base_;
  ulc_cmd_.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;

  // Set other fields to default values
  ulc_cmd_.clear = false;
  ulc_cmd_.enable_pedals = true;
  ulc_cmd_.enable_shifting = true;
  ulc_cmd_.enable_steering = true;
  ulc_cmd_.shift_from_park = true;
  ulc_cmd_.linear_accel = 0;
  ulc_cmd_.linear_decel = 0;
  ulc_cmd_.angular_accel = 0;
  ulc_cmd_.lateral_accel = 0;
  ulc_cmd_.jerk_limit_throttle=0;
  ulc_cmd_.jerk_limit_brake=0;
  //printf("ulc_cmd.linear_vel=%f", ulc_cmd_.linear_velocity);
}



void UbmkzInterface::callbackBrakeRpt(
  const ds_dbw_msgs::msg::BrakeReport::ConstSharedPtr rpt)
{
  sub_brake_ptr_ = rpt;
}

void UbmkzInterface::onControlModeRequest(
  const ControlModeCommand::Request::SharedPtr request,
  const ControlModeCommand::Response::SharedPtr response)
{
  if (request->mode == ControlModeCommand::Request::AUTONOMOUS) {
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  if (request->mode == ControlModeCommand::Request::MANUAL) {
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
  response->success = false;
  return;
}

void UbmkzInterface::callbackEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

void UbmkzInterface::recvDbwEnabled(std_msgs::msg::Bool::ConstSharedPtr msg){
  dbw_enable_ = msg->data;
}


int32_t UbmkzInterface::toAutowareTurnIndicatorsReport(
  const ds_dbw_msgs::msg::Misc1Report &misc1_rpt)
{
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using ds_dbw_msgs::msg::Misc1Report;
  using ds_dbw_msgs::msg::TurnSignal;

  if (misc1_rpt.turn_signal.value == ds_dbw_msgs::msg::TurnSignal::RIGHT){
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (misc1_rpt.turn_signal.value == ds_dbw_msgs::msg::TurnSignal::LEFT){
        return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (misc1_rpt.turn_signal.value == ds_dbw_msgs::msg::TurnSignal::NONE){
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}



int32_t UbmkzInterface::toAutowareHazardLightsReport(
  const ds_dbw_msgs::msg::Misc1Report &misc1_rpt)
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
  using ds_dbw_msgs::msg::Misc1Report;
  using ds_dbw_msgs::msg::TurnSignal;

  if (misc1_rpt.turn_signal.value == ds_dbw_msgs::msg::TurnSignal::HAZARD) {
    return HazardLightsReport::ENABLE;
  }

  return HazardLightsReport::DISABLE;
}

void UbmkzInterface::publishCommands(){ 
  /* guard */
/*   if (sub_steering_ptr_ || sub_gear_ptr_ || sub_misc1_ptr_ || sub_ulc_rpt_ptr_){
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "sub_steering_ptr = %d, sub_gear_ptr= %d, sub_misc1_ptr=%d, sub_ulc_rpt_ptr=%d", 
      sub_steering_ptr_ !=nullptr, sub_gear_ptr_!=nullptr, sub_misc1_ptr_!=nullptr, sub_ulc_rpt_ptr_!=nullptr
    );
  } */
  pub_ulc_cmd_->publish(ulc_cmd_);

}


}