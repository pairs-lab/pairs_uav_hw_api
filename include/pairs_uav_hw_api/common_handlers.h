#ifndef COMMON_HANDLERS_H
#define COMMON_HANDLERS_H

#include <pairs_uav_hw_api/publishers.h>
#include <pairs_lib/transformer.h>

namespace pairs_uav_hw_api
{

//}

typedef std::function<std::string(void)> getUavName_t;
typedef std::function<std::string(void)> getBodyFrameName_t;
typedef std::function<std::string(void)> getWorldFrameName_t;

/**
 * @brief A structure with methods and variables provided to the HW API Plugin.
 */
struct CommonHandlers_t
{
  /**
   * @brief A structure with publishers that the plugin should use to provide data to the PAIRS UAV System.
   */
  Publishers_t publishers;

  /**
   * @brief The pairs_lib's TF2 transformer wrapper.
   */
  std::shared_ptr<pairs_lib::Transformer> transformer;

  /**
   * @brief Method for obtaining the current "UAV_NAME" (the one set from the $UAV_NAME env variable).
   */
  getUavName_t getUavName;

  /**
   * @brief Method for obtaining the body-fixed frame_id (as it is called by the PAIRS UAV System). This frame_id should be used when publishing data in the
   * body-fixed frame, e.g., the IMU data.
   */
  getBodyFrameName_t getBodyFrameName;

  /**
   * @brief Method for obtaining the world-frame for marking the flight-controller's data, e.g., the position of the UAV in the flight-controllers frame.
   */
  getWorldFrameName_t getWorldFrameName;
};

}  // namespace pairs_uav_hw_api

#endif  // COMMON_HANDLERS_H
