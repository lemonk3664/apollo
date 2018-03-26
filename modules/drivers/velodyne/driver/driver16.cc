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

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <string>
#include <thread>

#include "modules/drivers/velodyne/driver/driver.h"


namespace apollo {
namespace drivers {
namespace velodyne {

Velodyne16Driver::Velodyne16Driver(const Config &config) {
  config_ = config;
}

void Velodyne16Driver::init() {
  double packet_rate = 754;                 // packet frequency (Hz)
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int)ceil(packet_rate / frequency);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  // open Velodyne input device
  input_.reset(new SocketInput());
  positioning_input_.reset(new SocketInput());
  input_->init(config_.firing_data_port);
  positioning_input_->init(config_.positioning_data_port);
}

/** poll the device
 *
 *  @returns scan unless end of file reached
 */
velodyne_msgs::VelodyneScanUnified::ConstPtr Velodyne16Driver::poll(void) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanUnifiedPtr scan(
      new velodyne_msgs::VelodyneScanUnified);

  if (basetime_ == 0) {
    usleep(100);  // waiting for positioning data
    return nullptr;
  }

  int poll_result = poll_standard(scan);

  if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL) {
    return nullptr;  // poll again
  }

  if (scan->packets.empty()) {
    ROS_INFO_STREAM("Get a empty scan from port: " << config_.firing_data_port);
    return nullptr;
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = ros::Time().now();
  scan->header.frame_id = config_.frame_id;
  // we use fisrt packet gps time update gps base hour
  // in cloud nodelet, will update base time packet by packet
  uint32_t current_secs =
      *((uint32_t *)(&scan->packets.front().data[0] + 1200));
  update_gps_top_hour(current_secs);
  scan->basetime = basetime_;

  return scan;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
void Velodyne16Driver::poll_positioning_packet(void) {
  while (true) {
    NMEATimePtr nmea_time(new NMEATime);
    bool ret = true;
    while (true) {
      int rc = positioning_input_->get_positioning_data_packtet(nmea_time);
      if (rc == 0) {
        break;  // got a full packet
      }
      if (rc < 0) {
        ret = false;  // end of file reached
      }
    }

    if (basetime_ == 0 && ret) {
      set_base_time_from_nmea_time(nmea_time, basetime_);
    }
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo