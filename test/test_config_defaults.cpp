// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// config/config.yaml as the driver reads it (9B.1, 08 C.2 HesaiCharacterization.ConfigYaml-
// Defaults): the XT32 at 192.168.2.1, its topics and ports, the decoder and ROS switches, the
// stamping policy. A step that changes a default edits this pin in the same step.
#include <gtest/gtest.h>

#include <string>

#include "source_drive_common.hpp"

namespace {

struct Loaded {
  DriverParam driver;
  hesai_ros_driver::TimeSyncConfig sync;
};

Loaded LoadConfig() {
  const YAML::Node lidar = YAML::LoadFile(std::string(PROJECT_PATH) + "/config/config.yaml")["lidar"];
  Loaded loaded;
  DriveYamlParam yaml;
  yaml.GetDriveYamlParam(lidar[0], loaded.driver);
  DriveYamlParam::GetTimeSyncParam(lidar[0], loaded.sync);
  return loaded;
}

}  // namespace

TEST(HesaiCharacterization, ConfigYamlDefaults) {
  const Loaded c = LoadConfig();
  EXPECT_EQ(c.driver.input_param.frame_id, "xt32_lidar_link");
  EXPECT_EQ(c.driver.input_param.ros_send_point_topic, "/xt32/pcl");
  EXPECT_EQ(c.driver.input_param.ros_send_packet_loss_topic, "/lidar_packets_loss");
  EXPECT_EQ(c.driver.input_param.ros_send_packet_topic, "/lidar_packets");
  EXPECT_EQ(c.driver.input_param.ros_send_ptp_topic, "/xt32/ptp");
  EXPECT_EQ(c.driver.input_param.udp_port, 2368);
  EXPECT_EQ(c.driver.input_param.ptc_port, 9347);
  EXPECT_EQ(c.driver.input_param.device_ip_address, "192.168.2.1");
  EXPECT_EQ(c.driver.input_param.source_type, DATA_FROM_LIDAR);
  EXPECT_EQ(c.driver.decoder_param.use_timestamp_type, 0);
  EXPECT_FALSE(c.driver.decoder_param.enable_packet_loss_tool);
  EXPECT_TRUE(c.driver.input_param.send_point_cloud_ros);
  EXPECT_FALSE(c.driver.input_param.send_packet_ros);
  EXPECT_EQ(c.sync.source, hesai_ros_driver::TimestampSource::kAuto);
  EXPECT_TRUE(c.sync.publish_time_field);
}
