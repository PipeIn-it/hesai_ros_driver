/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: source_driver_ros2.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sstream>
#include <hesai_ros_driver/msg/udp_frame.hpp>
#include <hesai_ros_driver/msg/udp_packet.hpp>
#include <hesai_ros_driver/msg/ptp.hpp>
#include <hesai_ros_driver/msg/firetime.hpp>
#include <hesai_ros_driver/msg/loss_packet.hpp>

#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <boost/thread.hpp>
#include "source_drive_common.hpp"
#include "driver_accounting.hpp"

class SourceDriver
{
public:
  typedef std::shared_ptr<SourceDriver> Ptr;
  // Initialize with internally-created node (standalone mode)
  virtual void Init(const YAML::Node& config);
  // Initialize with externally-provided node (composed/component mode)
  virtual void Init(const YAML::Node& config, std::shared_ptr<rclcpp::Node> external_node);
  // Start working
  virtual void Start();
  // Stop working
  virtual void Stop();
  virtual ~SourceDriver();
  SourceDriver(SourceType src_type) {};
  void SpinRos2(){rclcpp::spin(this->node_ptr_);}
  std::shared_ptr<rclcpp::Node> node_ptr_;
  #ifdef __CUDACC__
    std::shared_ptr<HesaiLidarSdkGpu<LidarPointXYZIRT>> driver_ptr_;
  #else
    std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;
  #endif
protected:
  // Shared initialization logic used by both Init overloads
  void InitImpl(const YAML::Node& config, DriverParam& driver_param);
  // Save Correction file subscribed by "ros_recv_correction_topic"
  void RecieveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  // Save packets subscribed by 'ros_recv_packet_topic'
  void RecievePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg);
  // Used to publish point clouds through 'ros_send_point_cloud_topic'
  void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
  // Used to publish the original pcake through 'ros_send_packet_topic'
  void SendPacket(const UdpFrame_t&  ros_msg, double timestamp);

  // Used to publish the Correction file through 'ros_send_correction_topic'
  void SendCorrection(const u8Array_t& msg);
  // Used to publish the Packet loss condition
  void SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Used to publish the Packet loss condition
  void SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Used to publish the firetime correction 
  void SendFiretime(const double *firetime_correction_);

  // Convert ptp lock offset, status into ROS message
  hesai_ros_driver::msg::Ptp ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Convert packet loss condition into ROS message
  hesai_ros_driver::msg::LossPacket ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Convert correction string into ROS messages
  std_msgs::msg::UInt8MultiArray ToRosMsg(const u8Array_t& correction_string);
  // Convert double[512] to float64[512]
  hesai_ros_driver::msg::Firetime ToRosMsg(const double *firetime_correction_);
  // Convert point clouds into ROS messages
  sensor_msgs::msg::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
  // Convert packets into ROS messages
  hesai_ros_driver::msg::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg, double timestamp);
  std::string frame_id_;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_sub_;
  rclcpp::Subscription<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_sub_;
  rclcpp::Publisher<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Firetime>::SharedPtr firetime_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::LossPacket>::SharedPtr loss_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Ptp>::SharedPtr ptp_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  //spin thread while recieve data from ROS topic
  boost::thread* subscription_spin_thread_{nullptr};

  // Frame stamps: PTP (sensor) time while the lidar is locked, arrival time
  // otherwise. See time_sync_policy.hpp.
  hesai_ros_driver::TimeSyncConfig time_sync_cfg_;
  std::unique_ptr<hesai_ros_driver::TimeSyncPolicy> time_sync_;
  // 1 Hz status thread: polls PTP state over PTC (live lidar only) and
  // publishes the PTP topic and /diagnostics.
  void StatusLoop();
  void PublishDiagnostics();
  void StopStatusThread();
  bool poll_ptp_{false};
  std::thread status_thread_;
  std::mutex status_mutex_;
  std::condition_variable status_cv_;
  bool status_running_{false};
  bool composed_{false};  // true when running inside a component container
  // The frames published, and the ones the SDK skipped (9B).
  hesai_ros_driver::FrameAccounting frame_accounting_;
};

inline builtin_interfaces::msg::Time ToRos2Stamp(double seconds)
{
  const int64_t ns = static_cast<int64_t>(std::llround(seconds * 1e9));
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}
// Standalone mode: creates its own rclcpp::Node
inline void SourceDriver::Init(const YAML::Node& config)
{
  DriverParam driver_param;
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(config, driver_param);
  frame_id_ = driver_param.input_param.frame_id;

  node_ptr_.reset(new rclcpp::Node("hesai_ros_driver_node"));
  composed_ = false;
  InitImpl(config, driver_param);
}

// Composed mode: uses an externally-provided node (e.g. from a component container)
inline void SourceDriver::Init(const YAML::Node& config, std::shared_ptr<rclcpp::Node> external_node)
{
  DriverParam driver_param;
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(config, driver_param);
  frame_id_ = driver_param.input_param.frame_id;

  node_ptr_ = external_node;
  composed_ = true;
  InitImpl(config, driver_param);
}

// Shared initialization: creates publishers, subscribers, and SDK instance
inline void SourceDriver::InitImpl(const YAML::Node& config, DriverParam& driver_param)
{
  DriveYamlParam::GetTimeSyncParam(config, time_sync_cfg_);
  time_sync_.reset(new hesai_ros_driver::TimeSyncPolicy(time_sync_cfg_));
  poll_ptp_ = (driver_param.input_param.source_type == DATA_FROM_LIDAR);
  diag_pub_ = node_ptr_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  if (driver_param.input_param.send_point_cloud_ros) {
    pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(driver_param.input_param.ros_send_point_topic, 100);
  }


  if (hesai_ros_driver::LossTopicPolicy::ShouldAdvertise(
          driver_param.decoder_param.enable_packet_loss_tool,
          driver_param.input_param.ros_send_packet_loss_topic)) {
    loss_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      ptp_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Ptp>(driver_param.input_param.ros_send_ptp_topic, 10);
    }

    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      crt_pub_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, 10);
    }
  }
  if (! driver_param.input_param.firetimes_path.empty() ) {
    if (driver_param.input_param.ros_send_firetime_topic != NULL_TOPIC) {
      firetime_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Firetime>(driver_param.input_param.ros_send_firetime_topic, 10);
    }
  }

  if (driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET) {
    pkt_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_send_packet_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    pkt_sub_ = node_ptr_->create_subscription<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_recv_packet_topic, 10,
    std::bind(&SourceDriver::RecievePacket, this, std::placeholders::_1));
  if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {
    crt_sub_ = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_recv_correction_topic, 10,
    std::bind(&SourceDriver::RecieveCorrection, this, std::placeholders::_1));
  }
    driver_param.decoder_param.enable_udp_thread = false;
    // In composed mode, the container executor handles spinning — no dedicated thread needed
    if (!composed_) {
      subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos2,this));
    }
  }
  #ifdef __CUDACC__
    driver_ptr_.reset(new HesaiLidarSdkGpu<LidarPointXYZIRT>());
    driver_param.decoder_param.enable_parser_thread = false;
  #else
    driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
    driver_param.decoder_param.enable_parser_thread = true;
  #endif
  driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPointCloud, this, std::placeholders::_1));
  if(driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET){
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2)) ;
  }
  if (hesai_ros_driver::LossTopicPolicy::ShouldAdvertise(
          driver_param.decoder_param.enable_packet_loss_tool,
          driver_param.input_param.ros_send_packet_loss_topic)) {
  driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketLoss, this, std::placeholders::_1, std::placeholders::_2));
}
  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendCorrection, this, std::placeholders::_1));
}
    // PTP state is polled by StatusLoop, not by the SDK's frame loop: a PTC
    // timeout there would delay a frame, and the SDK only polls every 10 s.
  }
  if (!driver_ptr_->Init(driver_param))
  {
    std::cout << "Driver Initialize Error...." << std::endl;
    exit(-1);
  }
}

inline void SourceDriver::Start()
{
  driver_ptr_->Start();
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (!status_running_) {
    status_running_ = true;
    status_thread_ = std::thread(&SourceDriver::StatusLoop, this);
  }
}

inline SourceDriver::~SourceDriver()
{
  Stop();
}

inline void SourceDriver::Stop()
{
  StopStatusThread();
  driver_ptr_->Stop();
}

inline void SourceDriver::StopStatusThread()
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_running_ = false;
  }
  status_cv_.notify_all();
  if (status_thread_.joinable()) status_thread_.join();
}

inline void SourceDriver::StatusLoop()
{
  uint8_t lock_threshold_us = 255;  // PTC 0x3a: the lidar's configured lock threshold
  bool have_threshold = false;
  std::unique_lock<std::mutex> lock(status_mutex_);
  while (status_running_) {
    lock.unlock();
    if (poll_ptp_ && driver_ptr_->lidar_ptr_ != nullptr && driver_ptr_->lidar_ptr_->ptc_client_ != nullptr) {
      PtcClient* ptc = driver_ptr_->lidar_ptr_->ptc_client_;
      u8Array_t diag;
      hesai_ros_driver::PtpStatus status;
      if (ptc->GetPTPDiagnostics(diag, 1) == 0 &&
          hesai_ros_driver::DecodePtpStatusType1(diag.data(), diag.size(), &status)) {
        u8Array_t empty, lidar_status;
        if (ptc->QueryCommand(empty, lidar_status, kPTCGetLidarStatus) == 0) {
          status.clock_status = hesai_ros_driver::DecodePtpClockStatus(lidar_status.data(), lidar_status.size());
        }
        if (!have_threshold) {
          u8Array_t threshold;
          if (ptc->GetPTPLockOffset(threshold) == 0 && !threshold.empty()) {
            lock_threshold_us = threshold.front();
            have_threshold = true;
          }
        }
        time_sync_->UpdatePtpStatus(status, hesai_ros_driver::MonotonicNowSec());
        if (ptp_pub_) ptp_pub_->publish(ToRosMsg(lock_threshold_us, diag));
      } else {
        time_sync_->MarkPtcUnreachable();
      }
    }
    PublishDiagnostics();
    lock.lock();
    status_cv_.wait_for(lock, std::chrono::seconds(1), [this] { return !status_running_; });
  }
}

inline void SourceDriver::PublishDiagnostics()
{
  const hesai_ros_driver::TimeSyncSnapshot s = time_sync_->Snapshot(hesai_ros_driver::MonotonicNowSec());
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "hesai_ros_driver: " + frame_id_ + " timestamps";
  status.hardware_id = frame_id_;
  status.level = hesai_ros_driver::TimeSyncPolicy::Level(s, time_sync_cfg_) == hesai_ros_driver::DiagLevel::kOk
                     ? diagnostic_msgs::msg::DiagnosticStatus::OK
                     : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = hesai_ros_driver::TimeSyncPolicy::Summary(s, time_sync_cfg_);
  auto add = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  add("mode", hesai_ros_driver::StampModeName(s.mode));
  add("host_reason", hesai_ros_driver::HostReasonName(s.reason));
  add("tai_utc_offset_s", s.have_offset ? std::to_string(s.tai_offset_s) : "unknown");
  add("ptc_reachable", s.have_status ? (s.ptp.reachable ? "true" : "false") : "unknown");
  add("ptp_port_state", std::to_string(s.ptp.port_state));
  add("ptp_clock_status", hesai_ros_driver::PtpClockStatusName(s.ptp.clock_status));
  add("ptp_master_offset_ns", std::to_string(s.ptp.master_offset_ns));
  add("ptp_status_age_s", s.status_age_s >= 0.0 ? std::to_string(s.status_age_s) : "n/a");
  add("latency_ms", std::to_string(s.latency_s * 1e3));
  add("frame_span_ms", std::to_string(s.frame_span_s * 1e3));
  add("frames", std::to_string(s.frames));
  add("sensor_frames", std::to_string(s.sensor_frames));
  add("host_frames", std::to_string(s.host_frames));
  add("mode_transitions", std::to_string(s.transitions));
  add("implausible_frames", std::to_string(s.implausible_frames));
  add("non_monotonic_stamps", std::to_string(s.non_monotonic));
  {
    std::vector<std::pair<std::string, std::string>> frames;
    hesai_ros_driver::AppendFrameAccounting(frame_accounting_, &frames);
    for (const auto& kv : frames) add(kv.first, kv.second);
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_ptr_->now();
  array.status.push_back(status);
  diag_pub_->publish(array);
}

inline void SourceDriver::SendPacket(const UdpFrame_t& msg, double timestamp)
{
  pkt_pub_->publish(ToRosMsg(msg, timestamp));
}

inline void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{
  frame_accounting_.OnPublished(msg.frame_index);
  // Publish via unique_ptr to enable zero-copy when intra-process comms are active.
  // When not in a composed container, this is equivalent to the const-ref publish path.
  pub_->publish(std::make_unique<sensor_msgs::msg::PointCloud2>(ToRosMsg(msg, frame_id_)));
}

inline void SourceDriver::SendCorrection(const u8Array_t& msg)
{
  crt_pub_->publish(ToRosMsg(msg));
}

inline void SourceDriver::SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  loss_pub_->publish(ToRosMsg(total_packet_count, total_packet_loss_count));
}

inline void SourceDriver::SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  ptp_pub_->publish(ToRosMsg(ptp_lock_offset, ptp_status));
}

inline void SourceDriver::SendFiretime(const double *firetime_correction_)
{
  firetime_pub_->publish(ToRosMsg(firetime_correction_));
}

inline sensor_msgs::msg::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  sensor_msgs::msg::PointCloud2 ros_msg;

  // Fix 1: Empty frame guard — prevent UB on frame.points[0]
  if (frame.points_num == 0) {
    ros_msg.header.stamp = node_ptr_->now();
    ros_msg.header.frame_id = frame_id_;
    return ros_msg;
  }

  // Frame extent on the sensor clock: the stamp is chosen from it and point
  // times are written relative to its start.
  double t_first = frame.points[0].timestamp;
  double t_last = t_first;
  for (size_t i = 1; i < frame.points_num; i++) {
    const double t = frame.points[i].timestamp;
    if (t < t_first) t_first = t;
    if (t > t_last) t_last = t;
  }
  hesai_ros_driver::FrameTimes times;
  times.sensor_start_s = t_first;
  times.sensor_end_s = t_last;
  times.spin_rpm = frame.spin_speed;
  // node->now() follows use_sim_time, so bag replays align to the bag clock.
  const hesai_ros_driver::StampResult stamp =
      time_sync_->StampFrame(times, node_ptr_->now().seconds(), hesai_ros_driver::MonotonicNowSec());
  if (stamp.mode_changed) {
    RCLCPP_INFO(node_ptr_->get_logger(), "hesai_ros_driver: %s stamped with %s time%s%s", frame_id_.c_str(),
                hesai_ros_driver::StampModeName(stamp.mode),
                stamp.mode == hesai_ros_driver::StampMode::kHost ? ": " : "",
                stamp.mode == hesai_ros_driver::StampMode::kHost ? hesai_ros_driver::HostReasonName(stamp.reason) : "");
  }

  const bool publish_time = time_sync_cfg_.publish_time_field;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(publish_time ? 7 : 6);
  ros_msg.width = frame.points_num;
  ros_msg.height = 1;

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
  // Seconds from header.stamp, the field FAST-LIO and most deskewers read.
  if (publish_time) offset = addPointField(ros_msg, "time", 1, sensor_msgs::msg::PointField::FLOAT32, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = false;
  ros_msg.data.resize(frame.points_num * ros_msg.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_time_;
  if (publish_time) iter_time_.reset(new sensor_msgs::PointCloud2Iterator<float>(ros_msg, "time"));

  ros_msg.header.stamp = ToRos2Stamp(stamp.stamp_s);

  for (size_t i = 0; i < frame.points_num; i++)
  {
    LidarPointXYZIRT point = frame.points[i];
    // A frame that holds a clock step gets no point times: consumers then
    // derive them from the azimuth instead of using a multi-second offset.
    const double rel = stamp.point_times_valid ? point.timestamp - t_first : 0.0;
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.ring;
    *iter_timestamp_ = rel;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;
    if (iter_time_) {
      **iter_time_ = static_cast<float>(rel);
      ++(*iter_time_);
    }
  }
  ros_msg.header.frame_id = frame_id_;
  return ros_msg;
}

inline hesai_ros_driver::msg::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg, double timestamp) {
  hesai_ros_driver::msg::UdpFrame rs_msg;
  for (size_t i = 0 ; i < ros_msg.size(); i++) {
    hesai_ros_driver::msg::UdpPacket rawpacket;
    rawpacket.size = ros_msg[i].packet_len;
    rawpacket.data.resize(ros_msg[i].packet_len);
    memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
    rs_msg.packets.push_back(rawpacket);
  }
  // Same clock choice as the frame these packets belong to.
  rs_msg.header.stamp = ToRos2Stamp(time_sync_->StampPacket(timestamp, node_ptr_->now().seconds()));
  rs_msg.header.frame_id = frame_id_;
  return rs_msg;
}

inline std_msgs::msg::UInt8MultiArray SourceDriver::ToRosMsg(const u8Array_t& correction_string) {
  auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
  msg->data.resize(correction_string.size());
  std::copy(correction_string.begin(), correction_string.end(), msg->data.begin());
  return *msg;
}

inline hesai_ros_driver::msg::LossPacket SourceDriver::ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  hesai_ros_driver::msg::LossPacket msg;
  msg.total_packet_count = total_packet_count;
  msg.total_packet_loss_count = total_packet_loss_count;  
  return msg;
}

inline hesai_ros_driver::msg::Ptp SourceDriver::ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  hesai_ros_driver::msg::Ptp msg;
  msg.ptp_lock_offset = ptp_lock_offset;
  std::copy(ptp_status.begin(), ptp_status.begin() + std::min(16ul, ptp_status.size()), msg.ptp_status.begin());
  return msg;
}

inline hesai_ros_driver::msg::Firetime SourceDriver::ToRosMsg(const double *firetime_correction_)
{
  hesai_ros_driver::msg::Firetime msg;
  std::copy(firetime_correction_, firetime_correction_ + 512, msg.data.begin());
  return msg;
}
inline void SourceDriver::RecievePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg)
{
  uint64_t bag_ts_us = static_cast<uint64_t>(
    (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) * 1e6);
  for (size_t i = 0; i < msg->packets.size(); i++) {
    driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg->packets[i].data[0], msg->packets[i].size, bag_ts_us);
  }
}

inline void SourceDriver::RecieveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  driver_ptr_->lidar_ptr_->correction_string_.resize(msg->data.size());
  std::copy(msg->data.begin(), msg->data.end(), driver_ptr_->lidar_ptr_->correction_string_.begin());
  while (1) {
    if (! driver_ptr_->lidar_ptr_->LoadCorrectionFromROSbag()) {
      break;
    }
  }
}
