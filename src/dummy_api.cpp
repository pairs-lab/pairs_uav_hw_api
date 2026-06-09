/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <pairs_uav_hw_api/api.h>

#include <nav_msgs/msg/odometry.hpp>

#include <pairs_lib/param_loader.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/publisher_handler.h>

//}

namespace pairs_uav_hw_api
{

/* class DummyApi //{ */

class DummyApi : public pairs_uav_hw_api::PairsUavHwApi {

public:
  /* ~DummyApi(){}; */

  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers);

  void destroy();

  // | --------------------- status methods --------------------- |

  pairs_msgs::msg::HwApiStatus       getStatus();
  pairs_msgs::msg::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const pairs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  bool callbackControlGroupCmd(const pairs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  bool callbackAttitudeRateCmd(const pairs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  bool callbackAttitudeCmd(const pairs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgRateCmd(const pairs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgCmd(const pairs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgRateCmd(const pairs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgCmd(const pairs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  bool callbackPositionCmd(const pairs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);

  void callbackTrackerCmd(const pairs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | -------------------- service callbacks ------------------- |

  pairs_lib::Task<std::tuple<bool, std::string>> callbackArming(const bool &request);
  pairs_lib::Task<std::tuple<bool, std::string>> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void DummyApi::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers) {

  common_handlers_ = common_handlers;

  node_  = node;
  clock_ = node->get_clock();

  // | ------------------- loading parameters ------------------- |

  pairs_lib::ParamLoader param_loader(node_, "DummyHwApi");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "dummy HW API initialized");

  is_initialized_ = true;
}

//}

/* destroy() //{ */

void DummyApi::destroy() {
}

//}

/* getStatus() //{ */

pairs_msgs::msg::HwApiStatus DummyApi::getStatus() {

  pairs_msgs::msg::HwApiStatus diag;

  diag.stamp = clock_->now();

  diag.armed     = false;
  diag.offboard  = false;
  diag.connected = false;

  return diag;
}

//}

/* getCapabilities() //{ */

pairs_msgs::msg::HwApiCapabilities DummyApi::getCapabilities() {

  pairs_msgs::msg::HwApiCapabilities mode;

  mode.api_name = "DummyApi";
  mode.stamp    = clock_->now();

  return mode;
}

//}

/* callbackActuatorCmd() //{ */

bool DummyApi::callbackActuatorCmd([[maybe_unused]] const pairs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool DummyApi::callbackControlGroupCmd([[maybe_unused]] const pairs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool DummyApi::callbackAttitudeRateCmd([[maybe_unused]] const pairs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAttitudeCmd() //{ */

bool DummyApi::callbackAttitudeCmd([[maybe_unused]] const pairs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool DummyApi::callbackAccelerationHdgRateCmd([[maybe_unused]] const pairs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool DummyApi::callbackAccelerationHdgCmd([[maybe_unused]] const pairs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool DummyApi::callbackVelocityHdgRateCmd([[maybe_unused]] const pairs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool DummyApi::callbackVelocityHdgCmd([[maybe_unused]] const pairs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool DummyApi::callbackPositionCmd([[maybe_unused]] const pairs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void DummyApi::callbackTrackerCmd([[maybe_unused]] const pairs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {
}

//}

/* callbackArming() //{ */

pairs_lib::Task<std::tuple<bool, std::string>> DummyApi::callbackArming([[maybe_unused]] const bool &request) {

  co_return {false, "Dummy interface does not allow to arm."};
}

//}

/* callbackOffboard() //{ */

pairs_lib::Task<std::tuple<bool, std::string>> DummyApi::callbackOffboard(void) {

  co_return {false, "Dummy interface does not allow to switch to offboard."};
}

//}

} // namespace pairs_uav_hw_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pairs_uav_hw_api::DummyApi, pairs_uav_hw_api::PairsUavHwApi)
