/* includes //{ */

#include <ros/ros.h>

#include <pairs_uav_hw_api/api.h>

#include <nav_msgs/Odometry.h>

#include <pairs_lib/param_loader.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/subscribe_handler.h>

//}

namespace pairs_uav_hw_api
{

/* class DummyApi //{ */

class DummyApi : public pairs_uav_hw_api::PairsUavHwApi {

public:
  /* ~DummyApi(){}; */

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers);

  // | --------------------- status methods --------------------- |

  pairs_msgs::HwApiStatus       getStatus();
  pairs_msgs::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const pairs_msgs::HwApiActuatorCmd::ConstPtr msg);
  bool callbackControlGroupCmd(const pairs_msgs::HwApiControlGroupCmd::ConstPtr msg);
  bool callbackAttitudeRateCmd(const pairs_msgs::HwApiAttitudeRateCmd::ConstPtr msg);
  bool callbackAttitudeCmd(const pairs_msgs::HwApiAttitudeCmd::ConstPtr msg);
  bool callbackAccelerationHdgRateCmd(const pairs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg);
  bool callbackAccelerationHdgCmd(const pairs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg);
  bool callbackVelocityHdgRateCmd(const pairs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg);
  bool callbackVelocityHdgCmd(const pairs_msgs::HwApiVelocityHdgCmd::ConstPtr msg);
  bool callbackPositionCmd(const pairs_msgs::HwApiPositionCmd::ConstPtr msg);

  void callbackTrackerCmd(const pairs_msgs::TrackerCommand::ConstPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void DummyApi::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  pairs_lib::ParamLoader param_loader(nh_, "PairsUavHwApi");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PairsUavHwDummyApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[PairsUavHwDummyApi]: initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

pairs_msgs::HwApiStatus DummyApi::getStatus() {

  pairs_msgs::HwApiStatus diag;

  diag.stamp = ros::Time::now();

  diag.armed     = false;
  diag.offboard  = false;
  diag.connected = false;

  return diag;
}

//}

/* getCapabilities() //{ */

pairs_msgs::HwApiCapabilities DummyApi::getCapabilities() {

  pairs_msgs::HwApiCapabilities mode;

  mode.api_name = "DummyApi";
  mode.stamp    = ros::Time::now();

  return mode;
}

//}

/* callbackActuatorCmd() //{ */

bool DummyApi::callbackActuatorCmd([[maybe_unused]] const pairs_msgs::HwApiActuatorCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool DummyApi::callbackControlGroupCmd([[maybe_unused]] const pairs_msgs::HwApiControlGroupCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool DummyApi::callbackAttitudeRateCmd([[maybe_unused]] const pairs_msgs::HwApiAttitudeRateCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAttitudeCmd() //{ */

bool DummyApi::callbackAttitudeCmd([[maybe_unused]] const pairs_msgs::HwApiAttitudeCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool DummyApi::callbackAccelerationHdgRateCmd([[maybe_unused]] const pairs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool DummyApi::callbackAccelerationHdgCmd([[maybe_unused]] const pairs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool DummyApi::callbackVelocityHdgRateCmd([[maybe_unused]] const pairs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool DummyApi::callbackVelocityHdgCmd([[maybe_unused]] const pairs_msgs::HwApiVelocityHdgCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool DummyApi::callbackPositionCmd([[maybe_unused]] const pairs_msgs::HwApiPositionCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void DummyApi::callbackTrackerCmd([[maybe_unused]] const pairs_msgs::TrackerCommand::ConstPtr msg) {
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> DummyApi::callbackArming([[maybe_unused]] const bool &request) {

  return {false, "Dummy interface does not allow to arm."};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> DummyApi::callbackOffboard(void) {

  return {false, "Dummy interface does not allow to switch to offboard."};
}

//}

}  // namespace pairs_uav_hw_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pairs_uav_hw_api::DummyApi, pairs_uav_hw_api::PairsUavHwApi)
