#ifndef UBMKZ_INTERFACE__UBMKZ_INTERFACE_HPP_
#define UBMKZ_INTERFACE__UBMKZ_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>


// Messages
#include <can_msgs/msg/frame.hpp>
#include <ds_dbw_msg/msg/brake_cmd.hpp>
#include <ds_dbw_msg/msg/brake_info_report.hpp>
#include <ds_dbw_msg/msg/brake_report.hpp>
#include <ds_dbw_msg/msg/driver_assist_report.hpp>
#include <ds_dbw_msg/msg/fuel_level_report.hpp>
#include <ds_dbw_msg/msg/gear_cmd.hpp>
#include <ds_dbw_msg/msg/gear_report.hpp>
#include <ds_dbw_msg/msg/misc_cmd.hpp>
#include <ds_dbw_msg/msg/misc1_report.hpp>
#include <ds_dbw_msg/msg/steering_cmd.hpp>
#include <ds_dbw_msg/msg/steering_report.hpp>
#include <ds_dbw_msg/msg/surround_report.hpp>
#include <ds_dbw_msg/msg/throttle_cmd.hpp>
#include <ds_dbw_msg/msg/throttle_info_report.hpp>
#include <ds_dbw_msg/msg/throttle_report.hpp>
#include <ds_dbw_msg/msg/tire_pressure_report.hpp>
#include <ds_dbw_msg/msg/turn_signal.hpp>
#include <ds_dbw_msg/msg/wheel_position_report.hpp>
#include <ds_dbw_msg/msg/wheel_speed_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/empty.hpp>
#include <dataspeed_ulc_msgs/msg/ulc_cmd.hpp>
#include <dataspeed_ulc_msgs/msg/ulc_report.hpp>


// Platform & MOdule version map
#include <ds_dbw_can/PlatformMap.hpp>


// For Autoware
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>

#include <tier4_api_msgs/msg/door_status.hpp>
#include <tier4_external_api_msgs/srv/set_door.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

namespace ds_dbw_can {
class UbmkzInterface : public rclcpp::Node
{
public:
    using actuation_command_stamped = tier4_vehicle_msgs::msg::actuation_command_stamped;
    using actuation_status_stamped = tier4_vehicle_msgs::msg::actuation_status_stamped;
    using steering_wheel_status_stamped = tier4_vehicle_msgs::msg::steering_wheel_status_stamped;
    using control_mode_command = autoware_auto_vehicle_msgs::srv::control_mode_command;
    UbmkzInterface();

private:
    typedef message_filters::sync_policies::approximate_time<
        ds_dbw_msg::msg::steering_report, ds_dbw_msg::msg::gear_report,
        ds_dbw_msg::msg::misc1_report, dataspeed_ulc_msgs::msg::ulc_report>
        UbmkzFeedbackSynPolicy;

    /* Parameters */
    std::string base_frame_id_;
    int command_timeout_ms_;  // vehicle_cmd timeout [ms]
    bool dbw_enable_ =false;
    bool is_dbw_rpt_received_ = false;
    bool is_clear_override_needed_ = false;
    bool prev_override_ = false;
    double loop_rate_;           // [Hz]
    double wheel_base_;
    double steering_ratio_;
    double acker_wheelbase_;
    double track_width;

    /* Subscription */
    // from Autoware
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_control_cmd_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr sub_gear_cmd_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
        sub_turn_indicators_cmd_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
        sub_hazard_lights_cmd_;
    //rclcpp::Subscription<ActuationCommandStamped>::SharedPtr sub_actuation_cmd_;
    rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr sub_emergency_;
    
    // from dbwNode
    std::unique_ptr<message_filters::Subscriber<ds_dbw_msg::msg::SteeringReport>> sub_steering_;
    std::unique_ptr<message_filters::Subscriber<ds_dbw_msg::msg::GearReport>> sub_gear_;
    std::unique_ptr<message_filters::Subscriber<ds_dbw_msg::msg::Misc1Report>> sub_misc_1_;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
    rclcpp::Subscription<ds_dbw_msg::msg::BrakeReport>::SharedPtr sub_brake_;
    rclcpp::Subscription<ds_dbw_msg::msg::ThrottleReport>::SharedPtr sub_throttle_;    
    rclcpp::Subscription<ds_dbw_msg::msg::WheelSpeedReport>::SharedPtr sub_wheel_speeds_;
    rclcpp::Subscription<ds_dbw_msg::msg::WheelPositionReport>::SharedPtr sub_wheel_positions_;
    rclcpp::Subscription<ds_dbw_msg::msg::TirePressureReport>::SharedPtr sub_tire_pressure_;
    rclcpp::Subscription<ds_dbw_msg::msg::FuelLevelReport>::SharedPtr sub_fuel_level_;
    rclcpp::Subscription<ds_dbw_msg::msg::SurroundReport>::SharedPtr sub_surround_;
    rclcpp::Subscription<ds_dbw_msg::msg::BrakeInfoReport>::SharedPtr sub_brake_info_;
    rclcpp::Subscription<ds_dbw_msg::msg::ThrottleInfoReport>::SharedPtr sub_throttle_info_;
    rclcpp::Subscription<ds_dbw_msg::msg::DriverAssistReport>::SharedPtr sub_driver_assist_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sub_gps_time_;
    
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_gps_vel_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_vin_;      // Deprecated message
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_sys_enable_; // Deprecated message
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;

    // from UlcNode
    std::unique_ptr<message_filters::Subscriber<dataspeed_ulc_msgs::msg::UlcReport>> sub_ulc_rpt_;
    std::unique_ptr<message_filters::Synchronizer<BelivFeedbacksSyncPolicy>> ubmkz_feedbacks_sync_;


    /* Publisher */
    // To UlcNode
    rclcpp::Publisher<dataspeed_ulc_msgs::msg::UlcCmd>::SharedPtr pub_ulc_cmd_;


    // To Autoware
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr 
        pub_control_mode_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr 
        pub_vehicle_twist_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr 
        pub_steering_status_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr pub_gear_status_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr 
        pub_turn_indicators_status_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
        pub_hazard_lights_status_;
    //rclcpp::Publisher<ActuationStatusStamped>::SharedPtr pub_actuation_status_;
    rclcpp::Publisher<SteeringWheelStatusStamped>::SharedPtr pub_steering_wheel_status_;
    rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr pub_door_status_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    //service
    //tier4_api_utils::Service<tier4_external_api_msgs::srv::SetDoor>::SharedPtr srv_;
    rclcpp::Service<ControlModeCommand>::SharedPtr control_mode_server_;

    //dataspeed_ulc_msgs::msg::UlcCmd ulc_cmd_;



    /* input values */
    ActuationCommandStamped::ConstSharedPtr actuation_cmd_ptr_;
    autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
    autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
    autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
 
    ds_dbw_msg::msg::SteeringReport::ConstSharedPtr sub_steering_ptr_;
    ds_dbw_msg::msg::GearReport::ConstSharedPtr sub_gear_ptr_;
    ds_dbw_msg::msg::Misc1Report::ConstSharedPtr sub_misc1_ptr_;
    dataspeed_ulc_msgs::msg::UlcReport::ConstSharedPtr sub_ulc_rpt_ptr_;
    ds_dbw_msg::msg::BrakeReport::ConstSharedPtr sub_brake_ptr_;
    dataspeed_ulc_msgs::msg::UlcCmd ulc_cmd_;

    bool is_emergency_{false};
    rclcpp::Time control_command_received_time_;

   
    void callbackControlCmd(
        const autoware_auto_control_msgs::msg::AckermannControlCommand& msg);
    void callbackBrakeRpt(const ds_dbw_msg::msg::BrakeReport::ConstSharedPtr rpt); //verify once if error occurs
    void callbackInterface(
        const ds_dbw_msg::msg::SteeringReport::ConstSharedPtr steering_rpt,
        const ds_dbw_msg::msg::GearReport::ConstSharedPtr gear_rpt,
        const ds_dbw_msg::msg::Misc1Report::ConstSharedPtr misc1_rpt,
        const dataspeed_ulc_msgs::msg::UlcReport::ConstSharedPtr ulc_rpt);
    int32_t toAutowareShiftReport(const ds_dbw_msg::msg::GearReport& gear_rpt);
    int32_t toAutowareTurnIndicatorsReport(const ds_dbw_msg::msg::Misc1Report &misc1_rpt);
    int32_t toAutowareHazardLightsReport(const ds_dbw_msg::msg::Misc1Report &misc1_rpt);
    void onControlModeRequest(
        const ControlModeCommand::Request::SharedPtr request,
        const ControlModeCommand::Response::SharedPtr response);
    void callbackEmergencyCmd(
        const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
    void recvDbwEnabled(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void publishCommands();
};
}
#endif