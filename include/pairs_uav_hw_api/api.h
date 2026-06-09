#ifndef PAIRS_UAV_HW_API_H
#define PAIRS_UAV_HW_API_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <pairs_lib/subscriber_handler.h>

#include <pairs_uav_hw_api/common_handlers.h>

#include <pairs_msgs/msg/hw_api_actuator_cmd.hpp>
#include <pairs_msgs/msg/hw_api_control_group_cmd.hpp>
#include <pairs_msgs/msg/hw_api_attitude_cmd.hpp>
#include <pairs_msgs/msg/hw_api_attitude_rate_cmd.hpp>
#include <pairs_msgs/msg/hw_api_acceleration_hdg_rate_cmd.hpp>
#include <pairs_msgs/msg/hw_api_acceleration_hdg_cmd.hpp>
#include <pairs_msgs/msg/hw_api_velocity_hdg_rate_cmd.hpp>
#include <pairs_msgs/msg/hw_api_velocity_hdg_cmd.hpp>
#include <pairs_msgs/msg/hw_api_position_cmd.hpp>
#include <pairs_msgs/msg/hw_api_status.hpp>
#include <pairs_msgs/msg/hw_api_capabilities.hpp>
#include <pairs_msgs/msg/tracker_command.hpp>

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
   * @param node Node handle - Use this to create subscibers, publisher, etc.
   * @param common_handlers Structure pre-filled with useful variables, methods and objects that the plugin can use.
   */
  virtual void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers) = 0;

  virtual void destroy() = 0;

  /**
   * @brief Method for acquiring the HW API plugin's status. This method will be called repeatedly to obtain the status, which will be publisher by the plugin
   * manager.
   *
   * @return The status ROS message.
   */
  virtual pairs_msgs::msg::HwApiStatus getStatus() = 0;

  /**
   * @brief Method for obtaining the capabilities of the HW API Plugin. This method will be called repeatedly (after initialize()).
   *
   * @return The capabilities ROS message.
   */
  virtual pairs_msgs::msg::HwApiCapabilities getCapabilities() = 0;

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
  virtual bool callbackActuatorCmd(const pairs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming ControlGroupCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackControlGroupCmd(const pairs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming AttitudeRate+ThrottleCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAttitudeRateCmd(const pairs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming Attitude+ThrottleCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAttitudeCmd(const pairs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming Acceleration+HdgRateCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAccelerationHdgRateCmd(const pairs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming Acceleration+HdgCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackAccelerationHdgCmd(const pairs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming Velocity+HdgRateCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackVelocityHdgRateCmd(const pairs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming Velocity+HdgCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackVelocityHdgCmd(const pairs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming Velocity+PositionCmd control reference.
   *
   * @param msg incoming ROS message
   *
   * @return success (true if processed).
   */
  virtual bool callbackPositionCmd(const pairs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) = 0;

  /**
   * @brief Callback for the incoming TrackerCommand. The tracker command is provided by the reference generator of the PAIRS UAV System and it can be used as a
   * source for feed forward control action.
   *
   * @param msg incoming ROS message
   */
  virtual void callbackTrackerCmd(const pairs_msgs::msg::TrackerCommand::ConstSharedPtr msg) = 0;

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
  virtual pairs_lib::Task<std::tuple<bool, std::string>> callbackArming(const bool &request) = 0;

  /**
   * @brief Callback for a service call for switching the flight controller to the "offboard" mode. When in offboard mode, the flight controller is using the
   * provided reference commands for control.
   *
   * @return tuple(succes, message)
   */
  virtual pairs_lib::Task<std::tuple<bool, std::string>> callbackOffboard(void) = 0;

  /**
   * @brief Destructor
   */
  virtual ~PairsUavHwApi() = default;
};

} // namespace pairs_uav_hw_api

#endif
