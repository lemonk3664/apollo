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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/drivers/velodyne/velodyne.h"
#include "modules/drivers/velodyne/gflags/velodyne_gflags.h"
#include "modules/drivers/velodyne/driver/driver.h"
#include "modules/drivers/velodyne/pointcloud/compensator/compensator.h"
#include "modules/drivers/velodyne/pointcloud/cpmvert/convert.h"
/**
 * @namespace apollo::drivers::velodyne
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace velodyne {

//using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
//using apollo::common::monitor::MonitorMessageItem;
using apollo::common::ErrorCode;
//using apollo::common::time::Clock;
std::string Velodyne::Name() const {
  return FLAGS_driver_name;
}

apollo::common::Status Velodyne::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);
    AINFO << "The adapter manager is successfully initialized.";
  if (!::apollo::common::util::GetProtoFromFile(FLAGS_sensor_conf_file,
                                                &velodyne_conf_)) {
    return OnError("Unable to load canbus conf file: " +
                   FLAGS_sensor_conf_file);
    return Status::OK();
  }

  AINFO << "The canbus conf file is loaded: " << FLAGS_sensor_conf_file;
  ADEBUG << "velodyne_conf:" << velodyne_conf_.ShortDebugString();

  return Status::OK();
}

apollo::common::Status Velodyne::Start() {
  ROS_INFO("Velodyne node start");

  std::thread runner(&Velodyne::Run, this);
  runner.detach();

  // last step: publish monitor messages
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("Velodyne started.");
  return Status::OK();
}

void Run() {
  ros::init(argc, argv, FLAGS_node_namespace + "/" + FLAGS_sensor_node_name);
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  
  // create conversion class, which subscribes to raw data
  apollo::drivers::velodyne::Convert convert;
  convert.init(node, private_nh);
  auto driver = VelodyneDriverFactory::create_driver();
  Compensator compensator(node, velodyne_conf_);
  Convert convert;
  convert.init(velodyne_conf_);
  // loop until shut down or end of file
  while (ros::ok() && !stop_) {
    auto scan = driver->poll();
    if (scan != nullptr) {
      AdapterManager::PublishVelodynePackets(scan);
      convert.convert_packets_to_pointcloud(scan);
      auto compensted_pointcloud = compensator.pointcloud_callback(scan);
      if (compensted_pointcloud != nullptr) {
        AdapterManager::PublishVelodyneCompensatedPointCloud(compensted_pointcloud);
      }
    }
    ros::spinOnce();
  }
}

void Velodyne::Stop() {
  stop_ = true;
}

Status Velodyne::OnError(const std::string &error_msg) {
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
