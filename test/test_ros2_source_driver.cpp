// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// ROS 2 twin of test_ros1_source_driver.cpp (review of 9B, finding 6): the driver's own call
// sites of the 9B accounting (InitImpl's loss-topic decision, SendPointCloud, SendPacketLoss,
// PublishDiagnostics) through a J.14-style SourceDriver subclass, with InitImpl run for real
// against a live-lidar source that points at nothing (127.0.0.1, a closed PTC port, a free UDP
// port): the SDK finds no packet and returns, as on a robot whose lidar is still off.
// One case per process (the CMake registrations set GTEST_FILTER): the SDK's Logger singleton is
// stopped by every Lidar destructor and cannot stop twice. Written on the ROS 1 station; it is
// compiled and run on Jazzy.
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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "hesai_ros_driver/msg/loss_packet.hpp"
#include "source_driver_ros2.hpp"

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

/// One lidar entry of config.yaml, as the component passes it to Init(): a live lidar at
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

/// The driver as the component builds it, its protected call sites reachable. J.14: an SDK
/// object for the destructor's Stop() and the stamping policy exist without Init() too.
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
  size_t DiagnosticsSubscribers() const {
    return diag_pub_ ? diag_pub_->get_subscription_count() : 0;
  }
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

/// Messages of one topic, on a node of its own spun on its own thread.
template <typename M>
class Listener {
 public:
  explicit Listener(const std::string& topic) {
    static int count = 0;
    node_ = std::make_shared<rclcpp::Node>("test_listener_" + std::to_string(count++));
    sub_ = node_->create_subscription<M>(topic, rclcpp::QoS(100),
                                         [this](std::shared_ptr<M> msg) {
                                           std::lock_guard<std::mutex> lock(mutex_);
                                           messages_.push_back(*msg);
                                         });
    executor_.add_node(node_);
    thread_ = std::thread([this] { executor_.spin(); });
  }
  ~Listener() {
    executor_.cancel();
    thread_.join();
  }
  std::vector<M> All() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }
  size_t publishers() const { return sub_->get_publisher_count(); }

 private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Subscription<M>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
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

const std::string kStatusName = "hesai_ros_driver: test_lidar_link timestamps";

/// Publishes the driver's /diagnostics entry once; true when it arrived, into @p out.
bool PublishAndReceive(DriverUnderTest& driver,
                       Listener<diagnostic_msgs::msg::DiagnosticArray>& diagnostics,
                       diagnostic_msgs::msg::DiagnosticStatus* out) {
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

std::string ValueOf(const diagnostic_msgs::msg::DiagnosticStatus& status, const std::string& key) {
  for (const auto& kv : status.values) {
    if (kv.key == key) return kv.value;
  }
  return "<missing>";
}

class SourceDriverRos2 : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    // The SDK's logger writes ./log.log: in a directory of this test, never the cwd.
    char pattern[] = "/tmp/hesai_source_driver_XXXXXX";
    const char* dir = ::mkdtemp(pattern);
    if (dir != nullptr && ::chdir(dir) == 0) log_dir_ = dir;
  }
  static void TearDownTestSuite() {
    if (!log_dir_.empty()) {
      std::remove((log_dir_ + "/log.log").c_str());
      ::rmdir(log_dir_.c_str());
    }
    rclcpp::shutdown();
  }
  static std::string log_dir_;
};
std::string SourceDriverRos2::log_dir_;

}  // namespace

// H2 (9B.3) through InitImpl and the frame and diagnostics call sites: without the SDK's loss
// tool the loss topic is not advertised, though configured; the frames published with SDK
// indices 10, 11 and 15 count 3 published and 3 skipped on /diagnostics, and the packet totals
// read "n/a". At ab2af59 the topic was advertised and the entry had none of these keys.
TEST_F(SourceDriverRos2, WithoutTheLossToolNoLossTopicAndFramesCountedOnDiagnostics) {
  Listener<diagnostic_msgs::msg::DiagnosticArray> diagnostics("/diagnostics");
  auto node = std::make_shared<rclcpp::Node>("test_hesai_driver_a");
  DriverUnderTest driver;
  driver.Init(LidarConfig(/*loss_tool=*/false, "/test_hesai_a"), node);

  EXPECT_FALSE(driver.LossTopicAdvertised());
  for (int index : {10, 11, 15}) driver.SendPointCloud(*SdkFrame(index));
  diagnostic_msgs::msg::DiagnosticStatus status;
  ASSERT_TRUE(PublishAndReceive(driver, diagnostics, &status))
      << "no /diagnostics entry " << kStatusName;

  EXPECT_EQ(ValueOf(status, "frames_published"), "3");
  EXPECT_EQ(ValueOf(status, "frames_skipped"), "3");
  EXPECT_EQ(ValueOf(status, "packets_total"), "n/a");
  EXPECT_EQ(ValueOf(status, "packets_lost"), "n/a");
}

// H2 (9B.3): with the loss tool the topic is advertised and carries the SDK's totals, which also
// reach /diagnostics with the frame counts. At ab2af59 SendPacketLoss only published.
TEST_F(SourceDriverRos2, WithTheLossToolTheTotalsReachTheTopicAndDiagnostics) {
  Listener<diagnostic_msgs::msg::DiagnosticArray> diagnostics("/diagnostics");
  Listener<hesai_ros_driver::msg::LossPacket> loss("/test_hesai_b/packets_loss");
  auto node = std::make_shared<rclcpp::Node>("test_hesai_driver_b");
  DriverUnderTest driver;
  driver.Init(LidarConfig(/*loss_tool=*/true, "/test_hesai_b"), node);

  EXPECT_TRUE(driver.LossTopicAdvertised());
  ASSERT_TRUE(WaitFor([&] { return loss.publishers() > 0; }, 10.0)) << "no loss topic";
  for (int index : {10, 11, 15}) {
    driver.SendPointCloud(*SdkFrame(index));
    driver.SendPacketLoss(1000 + index, 7);  // the SDK reports its totals with each frame
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
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
