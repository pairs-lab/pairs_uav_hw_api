#define VERSION "2.0.0.0"

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <pairs_lib/node.h>
#include <pairs_lib/param_loader.h>
#include <pairs_lib/subscriber_handler.h>
#include <pairs_lib/transformer.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/timer_handler.h>
#include <pairs_lib/service_server_handler.h>
#include <pairs_lib/errorgraph/error_publisher.h>

#include <pairs_uav_hw_api/api.h>
#include <pairs_uav_hw_api/publishers.h>
#include <pairs_uav_hw_api/common_handlers.h>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <pairs_msgs/msg/float64_stamped.hpp>

#include <pluginlib/class_loader.hpp>

//}

/* using //{ */

using namespace std::chrono_literals;

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef pairs_lib::ROSTimer TimerType;
#else
typedef pairs_lib::ThreadTimer TimerType;
#endif

//}

namespace pairs_uav_hw_api
{

/* class HwApiManager //{ */

class HwApiManager : public pairs_lib::Node {

public:
  HwApiManager(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  std::string              _version_;
  std::atomic<bool>        is_initialized_ = false;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  void initialize();
  void shutdown();

  // | ----------------------- parameters ----------------------- |

  double _timer_diagnostics_rate_;
  double _timer_mode_rate_;

  double _pub_gnss_rate_;
  double _pub_gnss_status_rate_;
  double _pub_rtk_rate_;
  double _pub_imu_rate_;
  double _pub_distance_sensor_rate_;
  double _pub_altitude_rate_;
  double _pub_mag_heading_rate_;
  double _pub_mag_magnetic_field_rate_;
  double _pub_rc_channels_rate_;
  double _pub_battery_state_rate_;

  double _pub_position_rate_;
  double _pub_velocity_rate_;
  double _pub_orientation_rate_;
  double _pub_angular_velocity_rate_;
  double _pub_odometry_rate_;
  double _pub_ground_truth_rate_;

  std::string _plugin_address_;
  std::string _uav_name_;
  std::string _body_frame_name_;
  std::string _world_frame_name_;
  std::string _topic_prefix_;

  // | ---------------------- errorgraph --------------------- |

  enum class error_type_t : uint16_t
  {
    version_mismatch,
    parameter_loading,
    not_connected,
  };

  std::unique_ptr<pairs_lib::errorgraph::ErrorPublisher> error_publisher_;

  // | ---------------------- param loader ---------------------- |

  std::shared_ptr<pairs_lib::ParamLoader> param_loader_;

  // | ----------------------- transformer ---------------------- |

  std::shared_ptr<pairs_lib::Transformer> transformer_;

  // | ---------------------- plugin loader --------------------- |

  std::unique_ptr<pluginlib::ClassLoader<pairs_uav_hw_api::PairsUavHwApi>> plugin_loader_;
  std::shared_ptr<pairs_uav_hw_api::PairsUavHwApi>                         hw_api_;

  // | --------------------- common handlers -------------------- |

  // contains handlers that are shared with trackers and controllers
  // safety area, tf transformer, scope timer logger, and bumper
  std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiActuatorCmd>            sh_actuator_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiControlGroupCmd>        sh_control_group_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAttitudeRateCmd>        sh_attitude_rate_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAttitudeCmd>            sh_attitude_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAccelerationHdgRateCmd> sh_acceleration_hdg_rate_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAccelerationHdgCmd>     sh_acceleration_hdg_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiVelocityHdgRateCmd>     sh_velocity_hdg_rate_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiVelocityHdgCmd>         sh_velocity_hdg_cmd_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiPositionCmd>            sh_position_cmd_;

  pairs_lib::SubscriberHandler<pairs_msgs::msg::TrackerCommand> sh_tracker_cmd_;

  void callbackActuatorCmd(const pairs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  void callbackControlGroupCmd(const pairs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  void callbackAttitudeRateCmd(const pairs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  void callbackAttitudeCmd(const pairs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  void callbackAccelerationHdgRateCmd(const pairs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  void callbackAccelerationHdgCmd(const pairs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  void callbackVelocityHdgRateCmd(const pairs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  void callbackVelocityHdgCmd(const pairs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  void callbackPositionCmd(const pairs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);
  void callbackTrackerCmd(const pairs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | ----------------------- publishers ----------------------- |

  pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiStatus>       ph_status_;
  pairs_lib::PublisherHandler<std_msgs::msg::Empty>             ph_connected_;
  pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiCapabilities> ph_capabilities_;

  pairs_lib::PublisherHandler<sensor_msgs::msg::NavSatFix>     ph_gnss_;
  pairs_lib::PublisherHandler<pairs_msgs::msg::GpsInfo>          ph_gnss_status_;
  pairs_lib::PublisherHandler<pairs_msgs::msg::RtkGps>           ph_rtk_;
  pairs_lib::PublisherHandler<sensor_msgs::msg::Imu>           ph_imu_;
  pairs_lib::PublisherHandler<sensor_msgs::msg::Range>         ph_distance_sensor_;
  pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiAltitude>    ph_altitude_;
  pairs_lib::PublisherHandler<pairs_msgs::msg::Float64Stamped>   ph_mag_heading_;
  pairs_lib::PublisherHandler<sensor_msgs::msg::MagneticField> ph_mag_magnetic_field_;
  pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiRcChannels>  ph_rc_channels_;
  pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiRcRssi>      ph_rc_rssi_;
  pairs_lib::PublisherHandler<sensor_msgs::msg::BatteryState>  ph_battery_state_;

  pairs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>      ph_position_;
  pairs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>    ph_velocity_;
  pairs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped> ph_orientation_;
  pairs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>    ph_angular_velocity_;
  pairs_lib::PublisherHandler<nav_msgs::msg::Odometry>               ph_odometry_;
  pairs_lib::PublisherHandler<nav_msgs::msg::Odometry>               ph_ground_truth_;

  std::string getUavName(void);
  std::string getBodyFrameName(void);
  std::string getWorldFrameName(void);

  void publishGNSS(const sensor_msgs::msg::NavSatFix &msg);
  void publishGNSSStatus(const pairs_msgs::msg::GpsInfo &msg);
  void publishRTK(const pairs_msgs::msg::RtkGps &msg);
  void publishOdometry(const nav_msgs::msg::Odometry &msg);
  void publishGroundTruth(const nav_msgs::msg::Odometry &msg);
  void publishIMU(const sensor_msgs::msg::Imu &msg);
  void publishDistanceSensor(const sensor_msgs::msg::Range &msg);
  void publishAltitude(const pairs_msgs::msg::HwApiAltitude &msg);
  void publishMagnetometerHeading(const pairs_msgs::msg::Float64Stamped &msg);
  void publishMagneticField(const sensor_msgs::msg::MagneticField &msg);
  void publishStatus(const pairs_msgs::msg::HwApiStatus &msg);
  void publishRcChannels(const pairs_msgs::msg::HwApiRcChannels &msg);
  void publishRcRssi(const pairs_msgs::msg::HwApiRcRssi &msg);
  void publishOrientation(const geometry_msgs::msg::QuaternionStamped &msg);
  void publishPosition(const geometry_msgs::msg::PointStamped &msg);
  void publishVelocity(const geometry_msgs::msg::Vector3Stamped &msg);
  void publishAngularVelocity(const geometry_msgs::msg::Vector3Stamped &msg);
  void publishBatteryState(const sensor_msgs::msg::BatteryState &msg);

  // | ------------------------- timers ------------------------- |

  std::shared_ptr<TimerType> timer_diagnostics_;
  std::shared_ptr<TimerType> timer_mode_;

  void timerStatus(void);
  void timerMode(void);

  // | --------------------- service servers -------------------- |

  pairs_lib::ServiceServerHandler<std_srvs::srv::SetBool> ss_arming_;
  pairs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_offboard_;

  pairs_lib::Task<bool> callbackArming(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                     const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  pairs_lib::Task<bool> callbackOffboard(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // | ----------------------- publishers ----------------------- |
};

//}

/* HwApiManager() //{ */

HwApiManager::HwApiManager(rclcpp::NodeOptions options) : pairs_lib::Node("hw_api_manager", options) {

  this->initialize();
}

//}

/* initialize() //{ */

void HwApiManager::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::on_shutdown([this]() { this->shutdown(); });

  error_publisher_ = std::make_unique<pairs_lib::errorgraph::ErrorPublisher>(node_, clock_, "HwApiManager", "main");

  // | ----------------------- load params ---------------------- |

  param_loader_ = std::make_shared<pairs_lib::ParamLoader>(node_);

  std::vector<std::string> config_files;
  param_loader_->loadParam("configs", config_files);

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    param_loader_->addYamlFile(config_file);
  }

  param_loader_->loadParam("version", _version_);

  if (_version_ != VERSION) {

    RCLCPP_ERROR(node_->get_logger(), "the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    error_publisher_->addGeneralError(error_type_t::version_mismatch,
                                      "version mismatch: binary version " + std::string(VERSION) + " != config version " + _version_);

    error_publisher_->flushAndShutdown();
  }

  param_loader_->loadParam("hw_interface_plugin", _plugin_address_);
  param_loader_->loadParam("uav_name", _uav_name_);
  param_loader_->loadParam("body_frame_name", _body_frame_name_);
  param_loader_->loadParam("world_frame_name", _world_frame_name_);
  param_loader_->loadParam("topic_prefix", _topic_prefix_);
  param_loader_->loadParam("timers/diagnostics/rate", _timer_diagnostics_rate_);
  param_loader_->loadParam("timers/mode/rate", _timer_mode_rate_);

  param_loader_->loadParam("publish_rate/gnss", _pub_gnss_rate_);
  param_loader_->loadParam("publish_rate/gnss_status", _pub_gnss_status_rate_);
  param_loader_->loadParam("publish_rate/rtk", _pub_rtk_rate_);
  param_loader_->loadParam("publish_rate/imu", _pub_imu_rate_);
  param_loader_->loadParam("publish_rate/distance_sensor", _pub_distance_sensor_rate_);
  param_loader_->loadParam("publish_rate/altitude", _pub_altitude_rate_);
  param_loader_->loadParam("publish_rate/mag_heading", _pub_mag_heading_rate_);
  param_loader_->loadParam("publish_rate/mag_magnetic_field", _pub_mag_magnetic_field_rate_);
  param_loader_->loadParam("publish_rate/rc_channels", _pub_rc_channels_rate_);
  param_loader_->loadParam("publish_rate/battery_state", _pub_battery_state_rate_);

  param_loader_->loadParam("publish_rate/position", _pub_position_rate_);
  param_loader_->loadParam("publish_rate/velocity", _pub_velocity_rate_);
  param_loader_->loadParam("publish_rate/orientation", _pub_orientation_rate_);
  param_loader_->loadParam("publish_rate/angular_velocity", _pub_angular_velocity_rate_);
  param_loader_->loadParam("publish_rate/odometry", _pub_odometry_rate_);
  param_loader_->loadParam("publish_rate/ground_truth", _pub_ground_truth_rate_);

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    error_publisher_->addGeneralError(error_type_t::parameter_loading, "could not load all parameters");
    error_publisher_->flushAndShutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<pairs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = "HwApiManager";
  shopts.no_message_timeout                  = pairs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_actuator_cmd_ = pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiActuatorCmd>(shopts, "~/actuator_cmd", &HwApiManager::callbackActuatorCmd, this);

  sh_control_group_cmd_ =
      pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiControlGroupCmd>(shopts, "~/control_group_cmd", &HwApiManager::callbackControlGroupCmd, this);

  sh_attitude_cmd_ = pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAttitudeCmd>(shopts, "~/attitude_cmd", &HwApiManager::callbackAttitudeCmd, this);

  sh_attitude_rate_cmd_ =
      pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAttitudeRateCmd>(shopts, "~/attitude_rate_cmd", &HwApiManager::callbackAttitudeRateCmd, this);

  sh_acceleration_hdg_rate_cmd_ = pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAccelerationHdgRateCmd>(shopts, "~/acceleration_hdg_rate_cmd",
                                                                                                         &HwApiManager::callbackAccelerationHdgRateCmd, this);

  sh_acceleration_hdg_cmd_ =
      pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiAccelerationHdgCmd>(shopts, "~/acceleration_hdg_cmd", &HwApiManager::callbackAccelerationHdgCmd, this);

  sh_velocity_hdg_rate_cmd_ =
      pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiVelocityHdgRateCmd>(shopts, "~/velocity_hdg_rate_cmd", &HwApiManager::callbackVelocityHdgRateCmd, this);

  sh_velocity_hdg_cmd_ =
      pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiVelocityHdgCmd>(shopts, "~/velocity_hdg_cmd", &HwApiManager::callbackVelocityHdgCmd, this);

  sh_position_cmd_ = pairs_lib::SubscriberHandler<pairs_msgs::msg::HwApiPositionCmd>(shopts, "~/position_cmd", &HwApiManager::callbackPositionCmd, this);

  sh_tracker_cmd_ = pairs_lib::SubscriberHandler<pairs_msgs::msg::TrackerCommand>(shopts, "/" + _uav_name_ + "/control_manager/tracker_cmd",
                                                                              &HwApiManager::callbackTrackerCmd, this);

  // | ----------------------- publishers ----------------------- |


  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node = node_;

    ph_capabilities_ = pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiCapabilities>(opts, "~/capabilities");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node = node_;

    ph_status_ = pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiStatus>(opts, "~/status");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node = node_;

    ph_connected_ = pairs_lib::PublisherHandler<std_msgs::msg::Empty>(opts, "~/connected");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_gnss_rate_;

    ph_gnss_ = pairs_lib::PublisherHandler<sensor_msgs::msg::NavSatFix>(opts, "~/gnss");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_gnss_rate_;

    ph_gnss_status_ = pairs_lib::PublisherHandler<pairs_msgs::msg::GpsInfo>(opts, "~/gnss_status");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_rtk_rate_;

    ph_rtk_ = pairs_lib::PublisherHandler<pairs_msgs::msg::RtkGps>(opts, "~/rtk");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_distance_sensor_rate_;

    ph_distance_sensor_ = pairs_lib::PublisherHandler<sensor_msgs::msg::Range>(opts, "~/distance_sensor");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_mag_heading_rate_;

    ph_mag_heading_ = pairs_lib::PublisherHandler<pairs_msgs::msg::Float64Stamped>(opts, "~/mag_heading");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_mag_magnetic_field_rate_;

    ph_mag_magnetic_field_ = pairs_lib::PublisherHandler<sensor_msgs::msg::MagneticField>(opts, "~/magnetic_field");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_altitude_rate_;

    ph_altitude_ = pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiAltitude>(opts, "~/altitude");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_imu_rate_;

    ph_imu_ = pairs_lib::PublisherHandler<sensor_msgs::msg::Imu>(opts, "~/imu");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_rc_channels_rate_;

    ph_rc_channels_ = pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiRcChannels>(opts, "~/rc_channels");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_rc_channels_rate_;

    ph_rc_rssi_ = pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiRcRssi>(opts, "~/rc_rssi");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_battery_state_rate_;

    ph_battery_state_ = pairs_lib::PublisherHandler<sensor_msgs::msg::BatteryState>(opts, "~/battery_state");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_position_rate_;

    ph_position_ = pairs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>(opts, "~/position");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_orientation_rate_;

    ph_orientation_ = pairs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped>(opts, "~/orientation");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_velocity_rate_;

    ph_velocity_ = pairs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>(opts, "~/velocity");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_angular_velocity_rate_;

    ph_angular_velocity_ = pairs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>(opts, "~/angular_velocity");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_odometry_rate_;

    ph_odometry_ = pairs_lib::PublisherHandler<nav_msgs::msg::Odometry>(opts, "~/odometry");
  }

  {
    pairs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_ground_truth_rate_;

    ph_ground_truth_ = pairs_lib::PublisherHandler<nav_msgs::msg::Odometry>(opts, "~/ground_truth");
  }

  // | --------------------- service servers -------------------- |

  ss_arming_ =
      pairs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(node_, "~/arming", &HwApiManager::callbackArming, this, rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_offboard_ = pairs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(node_, "~/offboard", &HwApiManager::callbackOffboard, this, rclcpp::SystemDefaultsQoS(),
                                                                       cbkgrp_ss_);

  // | ------------------------- timers ------------------------- |

  pairs_lib::TimerHandlerOptions opts;

  opts.node           = node_;
  opts.autostart      = true;
  opts.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&HwApiManager::timerStatus, this);

    timer_diagnostics_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_timer_diagnostics_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&HwApiManager::timerMode, this);

    timer_mode_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_timer_mode_rate_, clock_), callback_fcn);
  }

  // | ---------------- bind the common handlers ---------------- |

  common_handlers_ = std::make_shared<pairs_uav_hw_api::CommonHandlers_t>();

  common_handlers_->main_param_loader = param_loader_;

  common_handlers_->transformer = transformer_;

  common_handlers_->getUavName        = std::bind(&HwApiManager::getUavName, this);
  common_handlers_->getBodyFrameName  = std::bind(&HwApiManager::getBodyFrameName, this);
  common_handlers_->getWorldFrameName = std::bind(&HwApiManager::getWorldFrameName, this);

  common_handlers_->publishers.publishGNSS                = std::bind(&HwApiManager::publishGNSS, this, std::placeholders::_1);
  common_handlers_->publishers.publishGNSSStatus          = std::bind(&HwApiManager::publishGNSSStatus, this, std::placeholders::_1);
  common_handlers_->publishers.publishRTK                 = std::bind(&HwApiManager::publishRTK, this, std::placeholders::_1);
  common_handlers_->publishers.publishDistanceSensor      = std::bind(&HwApiManager::publishDistanceSensor, this, std::placeholders::_1);
  common_handlers_->publishers.publishAltitude            = std::bind(&HwApiManager::publishAltitude, this, std::placeholders::_1);
  common_handlers_->publishers.publishIMU                 = std::bind(&HwApiManager::publishIMU, this, std::placeholders::_1);
  common_handlers_->publishers.publishMagnetometerHeading = std::bind(&HwApiManager::publishMagnetometerHeading, this, std::placeholders::_1);
  common_handlers_->publishers.publishMagneticField       = std::bind(&HwApiManager::publishMagneticField, this, std::placeholders::_1);
  common_handlers_->publishers.publishStatus              = std::bind(&HwApiManager::publishStatus, this, std::placeholders::_1);
  common_handlers_->publishers.publishRcChannels          = std::bind(&HwApiManager::publishRcChannels, this, std::placeholders::_1);
  common_handlers_->publishers.publishRcRssi              = std::bind(&HwApiManager::publishRcRssi, this, std::placeholders::_1);
  common_handlers_->publishers.publishBatteryState        = std::bind(&HwApiManager::publishBatteryState, this, std::placeholders::_1);

  common_handlers_->publishers.publishPosition        = std::bind(&HwApiManager::publishPosition, this, std::placeholders::_1);
  common_handlers_->publishers.publishOrientation     = std::bind(&HwApiManager::publishOrientation, this, std::placeholders::_1);
  common_handlers_->publishers.publishVelocity        = std::bind(&HwApiManager::publishVelocity, this, std::placeholders::_1);
  common_handlers_->publishers.publishAngularVelocity = std::bind(&HwApiManager::publishAngularVelocity, this, std::placeholders::_1);
  common_handlers_->publishers.publishOdometry        = std::bind(&HwApiManager::publishOdometry, this, std::placeholders::_1);
  common_handlers_->publishers.publishGroundTruth     = std::bind(&HwApiManager::publishGroundTruth, this, std::placeholders::_1);

  // | -------------------- load the plugin -------------------- |

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<pairs_uav_hw_api::PairsUavHwApi>>("pairs_uav_hw_api", "pairs_uav_hw_api::PairsUavHwApi");

  try {
    RCLCPP_INFO(node_->get_logger(), "loading the plugin '%s'", _plugin_address_.c_str());
    hw_api_ = plugin_loader_->createSharedInstance(_plugin_address_.c_str());
  }
  catch (pluginlib::CreateClassException &ex1) {
    RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the plugin '%s'", _plugin_address_.c_str());
    RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
    error_publisher_->addOneshotError("CreateClassException for the plugin " + _plugin_address_ + ": " + std::string(ex1.what()));
    error_publisher_->flushAndShutdown();
  }
  catch (pluginlib::PluginlibException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the plugin '%s'", _plugin_address_.c_str());
    RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
    error_publisher_->addOneshotError("PluginlibException for the plugin " + _plugin_address_ + ": " + std::string(ex.what()));
    error_publisher_->flushAndShutdown();
  }

  // | ------------------ initialize the plugin ----------------- |

  hw_api_->initialize(node_->create_sub_node("plugin"), common_handlers_);

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* shutdown() //{ */

void HwApiManager::shutdown() {

  std::cout << "HwApiManager: shutdown(): called" << std::endl;

  timer_diagnostics_->stop();
  timer_mode_->stop();

  std::cout << "HwApiManager: calling destroy() on hw api plugin" << std::endl;

  hw_api_->destroy();

  std::cout << "HwApiManager: unloading hw api plugin" << std::endl;

  hw_api_.reset();

  std::cout << "HwApiManager: finished shutdown()" << std::endl;
}

//}

// | --------------------- topic callbacks -------------------- |

/* callbackActuatorCmd() //{ */

void HwApiManager::callbackActuatorCmd(const pairs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackActuatorCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackControlGroupCmd() //{ */

void HwApiManager::callbackControlGroupCmd(const pairs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackControlGroupCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackAttitudeRateCmd() //{ */

void HwApiManager::callbackAttitudeRateCmd(const pairs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAttitudeRateCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'attitude rate' command!");
  }
}

//}

/* callbackAttitudeCmd() //{ */

void HwApiManager::callbackAttitudeCmd(const pairs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAttitudeCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'attitude' command!");
  }
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

void HwApiManager::callbackAccelerationHdgRateCmd(const pairs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAccelerationHdgRateCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'acceleration+hdg rate' command!");
  }
}

//}

/* callbackAccelerationHdgCmd() //{ */

void HwApiManager::callbackAccelerationHdgCmd(const pairs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAccelerationHdgCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'acceleration+hdg' command!");
  }
}

//}

/* callbackVelocityHdgRateCmd() //{ */

void HwApiManager::callbackVelocityHdgRateCmd(const pairs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackVelocityHdgRateCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'velocity+hdg rate' command!");
  }
}

//}

/* callbackVelocityHdgCmd() //{ */

void HwApiManager::callbackVelocityHdgCmd(const pairs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackVelocityHdgCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'velocity+hdg' command!");
  }
}

//}

/* callbackPositionCmd() //{ */

void HwApiManager::callbackPositionCmd(const pairs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackPositionCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'position' command!");
  }
}

//}

/* callbackTrackerCmd() //{ */

void HwApiManager::callbackTrackerCmd(const pairs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  hw_api_->callbackTrackerCmd(msg);
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackArming() //{ */

pairs_lib::Task<bool> HwApiManager::callbackArming(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                                 const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    co_return false;
  }

  RCLCPP_INFO(node_->get_logger(), "%s", request->data ? "arming" : "disarming");

  auto [success, message] = co_await hw_api_->callbackArming(request->data);

  response->success = success;
  response->message = message;

  co_return true;
}

//}

/* callbackOffboard() //{ */

pairs_lib::Task<bool> HwApiManager::callbackOffboard([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                   const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    co_return false;
  }

  RCLCPP_INFO(node_->get_logger(), "switching to offboard");

  auto [success, message] = co_await hw_api_->callbackOffboard();

  response->success = success;
  response->message = message;

  co_return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerStatus() //{ */

void HwApiManager::timerStatus() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerStatus() spinning");

  pairs_msgs::msg::HwApiStatus status = hw_api_->getStatus();

  ph_status_.publish(status);

  if (!status.connected) {
    error_publisher_->addGeneralError(error_type_t::not_connected, "not connected");
  }

  if (status.connected) {

    std_msgs::msg::Empty msg;
    ph_connected_.publish(msg);
  }
}

//}

/* timerMode() //{ */

void HwApiManager::timerMode() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerCapabilities() spinning");

  pairs_msgs::msg::HwApiCapabilities diag = hw_api_->getCapabilities();

  ph_capabilities_.publish(diag);
}

//}

// | --------------------- common handlers -------------------- |

/* getUavName() //{ */

std::string HwApiManager::getUavName(void) {

  return _uav_name_;
}

//}

/* getBodyFrameName() //{ */

std::string HwApiManager::getBodyFrameName(void) {

  return _body_frame_name_;
}

//}

/* getBodyFrameName() //{ */

std::string HwApiManager::getWorldFrameName(void) {

  return _world_frame_name_;
}

//}

// | ----------------------- publishers ----------------------- |

/* publishGNSS() //{ */

void HwApiManager::publishGNSS(const sensor_msgs::msg::NavSatFix &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_gnss_.publish(msg);
}

//}

/* publishGNSSStatus() //{ */

void HwApiManager::publishGNSSStatus(const pairs_msgs::msg::GpsInfo &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_gnss_status_.publish(msg);
}

//}

/* publishRTK() //{ */

void HwApiManager::publishRTK(const pairs_msgs::msg::RtkGps &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_rtk_.publish(msg);
}

//}

/* publishIMU() //{ */

void HwApiManager::publishIMU(const sensor_msgs::msg::Imu &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_imu_.publish(msg);
}

//}

/* publishDistanceSensor() //{ */

void HwApiManager::publishDistanceSensor(const sensor_msgs::msg::Range &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_distance_sensor_.publish(msg);
}

//}

/* publishAltitude() //{ */

void HwApiManager::publishAltitude(const pairs_msgs::msg::HwApiAltitude &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_altitude_.publish(msg);
}

//}

/* publishMagnetometerHeading() //{ */

void HwApiManager::publishMagnetometerHeading(const pairs_msgs::msg::Float64Stamped &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_mag_heading_.publish(msg);
}

//}

/* publishMagneticField() //{ */

void HwApiManager::publishMagneticField(const sensor_msgs::msg::MagneticField &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_mag_magnetic_field_.publish(msg);
}

//}

/* publishStatus() //{ */

void HwApiManager::publishStatus(const pairs_msgs::msg::HwApiStatus &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_status_.publish(msg);
}

//}

/* publishRcChannels() //{ */

void HwApiManager::publishRcChannels(const pairs_msgs::msg::HwApiRcChannels &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_rc_channels_.publish(msg);
}

//}

/* publishRcRssi() //{ */

void HwApiManager::publishRcRssi(const pairs_msgs::msg::HwApiRcRssi &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_rc_rssi_.publish(msg);
}

//}

/* publishPosition() //{ */

void HwApiManager::publishPosition(const geometry_msgs::msg::PointStamped &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_position_.publish(msg);
}

//}

/* publishOrientation() //{ */

void HwApiManager::publishOrientation(const geometry_msgs::msg::QuaternionStamped &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_orientation_.publish(msg);
}

//}

/* publishVelocity() //{ */

void HwApiManager::publishVelocity(const geometry_msgs::msg::Vector3Stamped &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_velocity_.publish(msg);
}

//}

/* publishAngularVelocity() //{ */

void HwApiManager::publishAngularVelocity(const geometry_msgs::msg::Vector3Stamped &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_angular_velocity_.publish(msg);
}

//}

/* publishOdometry() //{ */

void HwApiManager::publishOdometry(const nav_msgs::msg::Odometry &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_odometry_.publish(msg);
}

//}

/* publishGroundTruth() //{ */

void HwApiManager::publishGroundTruth(const nav_msgs::msg::Odometry &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_ground_truth_.publish(msg);
}

//}

/* publishBatteryState() //{ */

void HwApiManager::publishBatteryState(const sensor_msgs::msg::BatteryState &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_battery_state_.publish(msg);
}

//}

} // namespace pairs_uav_hw_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pairs_uav_hw_api::HwApiManager)
