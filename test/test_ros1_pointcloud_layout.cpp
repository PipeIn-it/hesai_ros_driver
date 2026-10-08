// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// The PointCloud2 the ROS1 driver publishes (9B.1, 08 C.2 HesaiCharacterization.Ros1Point-
// CloudLayout, J.14): its fields and offsets, the point times relative to the frame's first
// point, the frame id; an empty frame carries no fields.
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "source_driver_ros1.hpp"

namespace {

/// The driver without Init(): what ToRosMsg needs, set up as InitImpl() would (J.14: an SDK
/// object for the destructor's Stop(), the frame id, the stamping policy).
class LayoutDriver : public SourceDriver {
 public:
  LayoutDriver() : SourceDriver(DATA_FROM_LIDAR) {
    driver_ptr_ = std::make_shared<HesaiLidarSdk<LidarPointXYZIRT>>();
    frame_id_ = "xt32_lidar_link";
    time_sync_.reset(new hesai_ros_driver::TimeSyncPolicy(time_sync_cfg_));
  }
  using SourceDriver::ToRosMsg;
};

struct Field {
  std::string name;
  uint32_t offset;
  uint8_t datatype;
};

}  // namespace

TEST(HesaiCharacterization, Ros1PointCloudLayout) {
  ros::Time::init();
  LayoutDriver driver;
  LidarDecodedFrame<LidarPointXYZIRT> frame;
  frame.points_num = 1001;
  frame.spin_speed = 600;
  const double t0 = 1790000000.0;
  for (uint32_t i = 0; i < frame.points_num; ++i) {
    frame.points[i] = {1.0f + i, 2.0f, 3.0f, 40.0f, static_cast<uint16_t>(i % 32), t0 + i * 1e-5};
  }
  const sensor_msgs::PointCloud2 msg = driver.ToRosMsg(frame, "ignored");

  const std::vector<Field> expected{
      {"x", 0, sensor_msgs::PointField::FLOAT32},          {"y", 4, sensor_msgs::PointField::FLOAT32},
      {"z", 8, sensor_msgs::PointField::FLOAT32},          {"intensity", 12, sensor_msgs::PointField::FLOAT32},
      {"ring", 16, sensor_msgs::PointField::UINT16},       {"timestamp", 18, sensor_msgs::PointField::FLOAT64},
      {"time", 26, sensor_msgs::PointField::FLOAT32},
  };
  ASSERT_EQ(msg.fields.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(msg.fields[i].name, expected[i].name) << i;
    EXPECT_EQ(msg.fields[i].offset, expected[i].offset) << expected[i].name;
    EXPECT_EQ(msg.fields[i].datatype, expected[i].datatype) << expected[i].name;
    EXPECT_EQ(msg.fields[i].count, 1u) << expected[i].name;
  }
  EXPECT_EQ(msg.point_step, 30u);
  EXPECT_EQ(msg.height, 1u);
  EXPECT_EQ(msg.width, 1001u);
  EXPECT_FALSE(msg.is_dense);
  EXPECT_EQ(msg.header.frame_id, "xt32_lidar_link");
  sensor_msgs::PointCloud2ConstIterator<double> rel(msg, "timestamp");
  for (uint32_t i = 0; i < msg.width; ++i, ++rel) {
    ASSERT_NEAR(*rel, i * 1e-5, 5e-7)  // a double near 1.79e9 s is spaced 2.4e-7 s
        << "point " << i << ": relative to the first point";
  }
}

TEST(HesaiCharacterization, Ros1EmptyFrameHasNoFields) {
  ros::Time::init();
  LayoutDriver driver;
  LidarDecodedFrame<LidarPointXYZIRT> frame;
  frame.points_num = 0;
  const sensor_msgs::PointCloud2 msg = driver.ToRosMsg(frame, "ignored");
  EXPECT_TRUE(msg.fields.empty());
  EXPECT_EQ(msg.width, 0u);
  EXPECT_EQ(msg.header.frame_id, "xt32_lidar_link");
}
