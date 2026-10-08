// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// The PointCloud2 the ROS2 driver publishes (9B.1, J.26): the same fields and offsets as the
// ROS1 driver, point times relative to the frame's first point, the frame id.
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "source_driver_ros2.hpp"

namespace {

class LayoutDriver : public SourceDriver {
 public:
  LayoutDriver() : SourceDriver(DATA_FROM_LIDAR) {
    node_ptr_ = std::make_shared<rclcpp::Node>("layout_test");
    driver_ptr_ = std::make_shared<HesaiLidarSdk<LidarPointXYZIRT>>();
    frame_id_ = "xt32_lidar_link";
    time_sync_.reset(new hesai_ros_driver::TimeSyncPolicy(time_sync_cfg_));
  }
  using SourceDriver::ToRosMsg;
};

class Ros2Layout : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

}  // namespace

TEST_F(Ros2Layout, SameFieldsAsRos1) {
  LayoutDriver driver;
  LidarDecodedFrame<LidarPointXYZIRT> frame;
  frame.points_num = 1001;
  frame.spin_speed = 600;
  const double t0 = 1790000000.0;
  for (uint32_t i = 0; i < frame.points_num; ++i) {
    frame.points[i] = {1.0f + i, 2.0f, 3.0f, 40.0f, static_cast<uint16_t>(i % 32), t0 + i * 1e-5};
  }
  const sensor_msgs::msg::PointCloud2 msg = driver.ToRosMsg(frame, "ignored");
  using F = sensor_msgs::msg::PointField;
  const std::vector<std::pair<std::string, uint32_t>> expected{
      {"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}, {"ring", 16}, {"timestamp", 18}, {"time", 26}};
  ASSERT_EQ(msg.fields.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(msg.fields[i].name, expected[i].first);
    EXPECT_EQ(msg.fields[i].offset, expected[i].second);
  }
  EXPECT_EQ(msg.fields[4].datatype, F::UINT16);
  EXPECT_EQ(msg.fields[5].datatype, F::FLOAT64);
  EXPECT_EQ(msg.point_step, 30u);
  EXPECT_EQ(msg.header.frame_id, "xt32_lidar_link");
  sensor_msgs::PointCloud2ConstIterator<double> rel(msg, "timestamp");
  for (uint32_t i = 0; i < msg.width; ++i, ++rel) ASSERT_NEAR(*rel, i * 1e-5, 5e-7) << i;  // doubles near 1.79e9 s: 2.4e-7 s apart
}
