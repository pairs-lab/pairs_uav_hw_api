#ifndef PAIRS_UAV_HW_API_H
#define PAIRS_UAV_HW_API_H

/* includes //{ */

#include <ros/ros.h>

#include <pairs_lib/subscribe_handler.h>

#include <pairs_uav_hw_api/common_handlers.h>

#include <pairs_msgs/HwApiActuatorCmd.h>
#include <pairs_msgs/HwApiControlGroupCmd.h>
#include <pairs_msgs/HwApiAttitudeCmd.h>
#include <pairs_msgs/HwApiAttitudeRateCmd.h>
#include <pairs_msgs/HwApiAccelerationHdgRateCmd.h>
#include <pairs_msgs/HwApiAccelerationHdgCmd.h>
#include <pairs_msgs/HwApiVelocityHdgRateCmd.h>
#include <pairs_msgs/HwApiVelocityHdgCmd.h>
#include <pairs_msgs/HwApiPositionCmd.h>
#include <pairs_msgs/HwApiStatus.h>
#include <pairs_msgs/HwApiCapabilities.h>
#include <pairs_msgs/TrackerCommand.h>

//}

namespace pairs_uav_hw_api
{

/**
 * @brief Interface for the HW API Plugin.
 */
class PairsUavHwApi {

public:
  /**
   * @brief The initialization method. This will be called only once at the beginning of the lifetime of the plugin (after initialize()).
   *
   * @param parent_nh Node handle of the HW API plugin manager. Use this to create subscibers, publisher, etc.
   * @param common_handlers Structure pre-filled with useful variables, methods and objects that the plugin can use.
   */
  virtual void initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers) = 0;

  /**
   * @brief Method for acquiring the HW API plugin's status. This method will be called repeatedly to obtain the status, which will be publisher by the plugin
   * manager.
   *
   * @return The status ROS message.
   */
  virtual pairs_msgs::HwApiStatus getStatus() = 0;

  /**
   * @brief Method for obtaining the capabilities of the HW API Plugin. This method will be called repeatedly (after initialize()).
   *
   * @return The capabilities ROS message.
   */
  virtual pairs_msgs::HwApiCapabilities getCapabilities() = 0;

  // --------------------------------------------------------------
  // |                       topic callbacks                      |
  // --------------------------------------------------------------

  /**
   * @brief Callback for the incoming ActuatorCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackActuatorCmd(const pairs_msgs::HwApiActuatorCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming ControlGroupCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackControlGroupCmd(const pairs_msgs::HwApiControlGroupCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming AttitudeRate+ThrottleCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAttitudeRateCmd(const pairs_msgs::HwApiAttitudeRateCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming Attitude+ThrottleCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAttitudeCmd(const pairs_msgs::HwApiAttitudeCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming Acceleration+HdgRateCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAccelerationHdgRateCmd(const pairs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming Acceleration+HdgCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAccelerationHdgCmd(const pairs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming Velocity+HdgRateCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackVelocityHdgRateCmd(const pairs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming Velocity+HdgCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackVelocityHdgCmd(const pairs_msgs::HwApiVelocityHdgCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming Velocity+PositionCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackPositionCmd(const pairs_msgs::HwApiPositionCmd::ConstPtr msg) = 0;

  /**
   * @brief Callback for the incoming TrackerCommand. The tracker command is provided by the reference generator of the PAIRS UAV System and it can be used as a
   * source for feed forward control action.
   *
   * @param msg incoming ROS message
   */
  virtual void callbackTrackerCmd(const pairs_msgs::TrackerCommand::ConstPtr msg) = 0;

  // --------------------------------------------------------------
  // |                      service callbacks                     |
  // --------------------------------------------------------------

  /**
   * @brief Callback for a service call for arming/disarming the flight controller.
   *
   * @param request Arm/Disarm.
   *
   * @return tuple(succes, message)
   */
  virtual std::tuple<bool, std::string> callbackArming(const bool& request) = 0;

  /**
   * @brief Callback for a service call for switching the flight controller to the "offboard" mode. When in offboard mode, the flight controller is using the
   * provided reference commands for control.
   *
   * @return tuple(succes, message)
   */
  virtual std::tuple<bool, std::string> callbackOffboard(void) = 0;

  /**
   * @brief Destructor
   */
  virtual ~PairsUavHwApi() = default;
};

}  // namespace pairs_uav_hw_api

#endif
