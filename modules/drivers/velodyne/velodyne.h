/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/drivers/velodyne/proto/velodyne_conf.pb.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::common::Status;

/**
* @class ContiRadarCanbus
*
* @brief template of canbus-based sensor module main class (e.g., conti_radar).
*/
class Velodyne : public apollo::common::ApolloApp {
 public:
  Velodyne()
    : stop_(false), monitor_logger_(apollo::common::monitor::MonitorMessageItem::CANBUS) {}
  /**
  * @brief obtain module name
  * @return module name
  */
  std::string Name() const override;

  /**
  * @brief module initialization function
  * @return initialization status
  */
  apollo::common::Status Init() override;

  /**
  * @brief module start function
  * @return start status
  */
  apollo::common::Status Start() override;

  /**
  * @brief module stop function
  */
  void Stop() override;

 private:
  Status OnError(const std::string &error_msg);
  void Run();

  int64_t last_timestamp_ = 0;
  bool stop_;
  VelodyneConf velodyne_conf_;
  apollo::common::monitor::MonitorLogger monitor_logger_;
};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_RADAR_CONTI_RADAR_CONTI_RADAR_CANBUS_H_
