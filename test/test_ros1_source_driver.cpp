// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// The ROS1 driver's own call sites of the 9B accounting (review of 9B, finding 6): InitImpl's
// loss-topic decision, SendPointCloud's frame accounting, SendPacketLoss's packet totals and the
// keys PublishDiagnostics adds, through a J.14-style SourceDriver subclass. InitImpl runs for
// real with a live-lidar source that points at nothing (127.0.0.1, a closed PTC port, an
// ephemeral UDP port): the SDK finds no packet and returns, as on a robot whose lidar is still
// off. The SDK object's own frame loop is not started.
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "hesai_ros_driver/LossPacket.h"
#include "source_driver_ros1.hpp"

namespace {

/// A free ephemeral port of @p type (SOCK_STREAM, SOCK_DGRAM): bound once to learn it, then
/// closed. Never a real lidar's (2368, 9347), and nothing sends to or listens on it.
uint16_t FreeEphemeralPort(int type) {
  const int fd = ::socket(AF_INET, type, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  socklen_t len = sizeof(addr);
  uint16_t port = 0;
  if (fd >= 0 && ::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0 &&
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0) {
    port = ntohs(addr.sin_port);
  }
  if (fd >= 0) ::close(fd);
  return port;
}

/// One lidar entry of config.yaml, as the nodelet passes it to Init(): a live lidar at
/// 127.0.0.1, its point cloud and packet-loss topics under @p ns.
YAML::Node LidarConfig(bool loss_tool, const std::string& ns) {
  YAML::Node config;
  config["driver"]["source_type"] = 1;
  config["driver"]["device_ip_address"] = "127.0.0.1";
  config["driver"]["ptc_port"] = FreeEphemeralPort(SOCK_STREAM);  // closed: PTC is refused
  // A port number, not 0: the SDK's socket source reopens a socket it sees on port 0.
  config["driver"]["udp_port"] = FreeEphemeralPort(SOCK_DGRAM);
  config["driver"]["enable_packet_loss_tool"] = loss_tool;
  config["ros"]["ros_frame_id"] = "test_lidar_link";
  config["ros"]["send_point_cloud_ros"] = true;
  config["ros"]["ros_send_point_cloud_topic"] = ns + "/points";
  config["ros"]["ros_send_packet_loss_topic"] = ns + "/packets_loss";
  return config;
}

/// The driver as the nodelet builds it, its protected call sites reachable. J.14: an SDK object
/// for the destructor's Stop() and the stamping policy exist without Init() too.
class DriverUnderTest : public SourceDriver {
 public:
  DriverUnderTest() : SourceDriver(DATA_FROM_LIDAR) {
    driver_ptr_ = std::make_shared<HesaiLidarSdk<LidarPointXYZIRT>>();
    time_sync_.reset(new hesai_ros_driver::TimeSyncPolicy(time_sync_cfg_));
  }
  using SourceDriver::PublishDiagnostics;
  using SourceDriver::SendPacketLoss;
  using SourceDriver::SendPointCloud;
  bool LossTopicAdvertised() const { return static_cast<bool>(loss_pub_); }
  uint32_t DiagnosticsSubscribers() const { return diag_pub_.getNumSubscribers(); }
};

/// A frame of 1,001 points (more than kMinPointsOfOneFrame) with the SDK's frame index @p index.
std::unique_ptr<LidarDecodedFrame<LidarPointXYZIRT>> SdkFrame(int index) {
  auto frame = std::make_unique<LidarDecodedFrame<LidarPointXYZIRT>>();
  frame->frame_index = index;
  frame->points_num = 1001;
  frame->spin_speed = 600;
  const double t0 = 1790000000.0 + index * 0.1;
  for (uint32_t i = 0; i < frame->points_num; ++i) {
    frame->points[i] = {1.0f, 2.0f, 3.0f, 40.0f, static_cast<uint16_t>(i % 32), t0 + i * 1e-5};
  }
  return frame;
}

/// Messages of one topic, on their own queue and thread.
template <typename M>
class Listener {
 public:
  explicit Listener(const std::string& topic) {
    nh_.setCallbackQueue(&queue_);
    sub_ = nh_.subscribe<M>(topic, 100, &Listener::OnMessage, this);
    spinner_.start();
  }
  ~Listener() {
    spinner_.stop();
    sub_.shutdown();
  }
  std::vector<M> All() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }
  bool connected() const { return sub_.getNumPublishers() > 0; }

 private:
  void OnMessage(const typename M::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    messages_.push_back(*msg);
  }
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::AsyncSpinner spinner_{1, &queue_};
  ros::Subscriber sub_;
  mutable std::mutex mutex_;
  std::vector<M> messages_;
};

template <typename F>
bool WaitFor(F done, double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

bool TopicAdvertised(const std::string& topic) {
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  for (const auto& info : topics) {
    if (info.name == topic) return true;
  }
  return false;
}

const std::string kStatusName = "hesai_ros_driver: test_lidar_link timestamps";

/// Publishes the driver's /diagnostics entry once; true when it arrived, into @p out.
bool PublishAndReceive(DriverUnderTest& driver,
                       Listener<diagnostic_msgs::DiagnosticArray>& diagnostics,
                       diagnostic_msgs::DiagnosticStatus* out) {
  if (!WaitFor([&] { return driver.DiagnosticsSubscribers() > 0; }, 10.0)) return false;
  const size_t before = diagnostics.All().size();
  driver.PublishDiagnostics();
  if (!WaitFor([&] { return diagnostics.All().size() > before; }, 5.0)) return false;
  for (const auto& array : diagnostics.All()) {
    for (const auto& status : array.status) {
      if (status.name == kStatusName) {
        *out = status;
        return true;
      }
    }
  }
  return false;
}

std::string ValueOf(const diagnostic_msgs::DiagnosticStatus& status, const std::string& key) {
  for (const auto& kv : status.values) {
    if (kv.key == key) return kv.value;
  }
  return "<missing>";
}

}  // namespace

// H2 (9B.3) through InitImpl and the frame and diagnostics call sites: without the SDK's loss
// tool the loss topic is not advertised, though configured (the SDK would never fill it); the
// frames published with SDK indices 10, 11 and 15 count 3 published and 3 skipped on the 1 Hz
// /diagnostics entry, and the packet totals read "n/a". At ab2af59 the topic was advertised and
// the entry had none of these keys.
TEST(SourceDriverRos1, WithoutTheLossToolNoLossTopicAndFramesCountedOnDiagnostics) {
  Listener<diagnostic_msgs::DiagnosticArray> diagnostics("/diagnostics");
  DriverUnderTest driver;
  driver.Init(LidarConfig(/*loss_tool=*/false, "/test_hesai_a"),
              std::make_shared<ros::NodeHandle>());

  EXPECT_FALSE(driver.LossTopicAdvertised());
  EXPECT_FALSE(TopicAdvertised("/test_hesai_a/packets_loss"));
  EXPECT_TRUE(TopicAdvertised("/test_hesai_a/points"));
  for (int index : {10, 11, 15}) driver.SendPointCloud(*SdkFrame(index));
  diagnostic_msgs::DiagnosticStatus status;
  ASSERT_TRUE(PublishAndReceive(driver, diagnostics, &status))
      << "no /diagnostics entry " << kStatusName;

  EXPECT_EQ(ValueOf(status, "frames_published"), "3");
  EXPECT_EQ(ValueOf(status, "frames_skipped"), "3");
  EXPECT_EQ(ValueOf(status, "packets_total"), "n/a");
  EXPECT_EQ(ValueOf(status, "packets_lost"), "n/a");
}

// H2 (9B.3): with the loss tool the topic is advertised and carries the SDK's totals, which also
// reach /diagnostics with the frame counts. At ab2af59 SendPacketLoss only published.
TEST(SourceDriverRos1, WithTheLossToolTheTotalsReachTheTopicAndDiagnostics) {
  Listener<diagnostic_msgs::DiagnosticArray> diagnostics("/diagnostics");
  Listener<hesai_ros_driver::LossPacket> loss("/test_hesai_b/packets_loss");
  DriverUnderTest driver;
  driver.Init(LidarConfig(/*loss_tool=*/true, "/test_hesai_b"),
              std::make_shared<ros::NodeHandle>());

  EXPECT_TRUE(driver.LossTopicAdvertised());
  ASSERT_TRUE(WaitFor([&] { return loss.connected(); }, 10.0)) << "no loss topic";
  for (int index : {10, 11, 15}) {
    driver.SendPointCloud(*SdkFrame(index));
    driver.SendPacketLoss(1000 + index, 7);  // the SDK reports its totals with each frame
  }
  diagnostic_msgs::DiagnosticStatus status;
  ASSERT_TRUE(PublishAndReceive(driver, diagnostics, &status))
      << "no /diagnostics entry " << kStatusName;

  EXPECT_EQ(ValueOf(status, "frames_published"), "3");
  EXPECT_EQ(ValueOf(status, "frames_skipped"), "3");
  EXPECT_EQ(ValueOf(status, "packets_total"), "1015");
  EXPECT_EQ(ValueOf(status, "packets_lost"), "7");
  ASSERT_TRUE(WaitFor([&] { return loss.All().size() >= 3; }, 5.0));
  EXPECT_EQ(loss.All().back().total_packet_count, 1015u);
  EXPECT_EQ(loss.All().back().total_packet_loss_count, 7u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros1_source_driver");
  ros::NodeHandle nh;
  // The SDK's logger writes ./log.log: in a directory of this test, never the node's cwd
  // (ROS_HOME under rostest, shared with a station driver).
  char pattern[] = "/tmp/hesai_source_driver_XXXXXX";
  const char* dir = ::mkdtemp(pattern);
  if (dir == nullptr || ::chdir(dir) != 0) return 2;
  const int result = RUN_ALL_TESTS();
  std::remove((std::string(dir) + "/log.log").c_str());
  ::rmdir(dir);
  return result;
}
