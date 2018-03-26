/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/velodyne/pointcloud/convert/convert.h"

#include <pcl/common/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/advertise_options.h>

namespace apollo {
namespace drivers {
namespace velodyne {

// void disconnected(const ros::SingleSubscriberPublisher&) {}
// void connected(const ros::SingleSubscriberPublisher&) {}

void Convert::init(const VelodyneConf& velodyne_conf) {
  config_.max_range = velodyne_conf.max_range();
  config_.min_range = velodyne_conf.min_range();
  config_.view_direction = velodyne_conf.view_direction();
  if (!velodyne_conf.has_view_width()) {
    config_.view_width = 2 * M_PI;
  } else {
    config_.view_width = velodyne_conf.view_width();
  }
  config_.model = velodyne_conf.model();
  config_.calibration_online = velodyne_conf.velodyne64_calibration_online();
  config_.calibration_file = velodyne_conf.velodyne64_calibration_file();
  config_.organized = velodyne_conf.organized();
  queue_size_ = 10;

  parser_ = VelodyneParserFactory::create_parser(config_);
  if (parser_ == nullptr) {
    ROS_BREAK();
  }
  parser_->setup();
}

Convert::~Convert() {
  if (parser_ != nullptr) {
    delete parser_;
  }
}

/** @brief Callback for raw scan messages. */
void Convert::convert_packets_to_pointcloud(
    const velodyne_msgs::VelodyneScanUnified::ConstPtr& scan_msg) {
  ROS_INFO_ONCE("********************************************************");
  ROS_INFO_ONCE("Start convert velodyne packets to pointcloud");
  ROS_INFO_ONCE("********************************************************");
  ROS_DEBUG_STREAM(scan_msg->header.seq);

  VPointCloud::Ptr pointcloud(new VPointCloud());
  parser_->generate_pointcloud(scan_msg, pointcloud);

  if (pointcloud->empty()) {
    return;
  }

  if (config_.organized) {
    ROS_DEBUG_STREAM("reorder point cloud");
    parser_->order(pointcloud);
  }

  // publish the accumulated cloud message
  pointcloud_pub_.publish(pointcloud);
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
