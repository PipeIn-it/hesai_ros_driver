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
 * File: hesai_driver_component.cpp
 * Description: ROS2 component wrapper for the Hesai LiDAR driver.
 *              Allows the driver to be loaded into a component container
 *              alongside other components (e.g. filter, assembler) for
 *              zero-copy intra-process communication.
 */

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "source_driver_ros2.hpp"

namespace hesai_ros_driver {

class HesaiDriverComponent : public rclcpp::Node {
public:
  explicit HesaiDriverComponent(const rclcpp::NodeOptions& options)
      : rclcpp::Node("hesai_ros_driver_node", options) {
    // Defer initialization — shared_from_this() is not available in the constructor.
    using namespace std::chrono_literals;
    init_timer_ = create_wall_timer(0ms, [this]() {
      init_timer_->cancel();

      declare_parameter("config_path", "");
      std::string config_path = get_parameter("config_path").as_string();
      if (config_path.empty()) {
        config_path = ament_index_cpp::get_package_share_directory("hesai_ros_driver")
                      + "/config/config.yaml";
      }

      RCLCPP_INFO(get_logger(), "Loading Hesai config from: %s", config_path.c_str());
      YAML::Node config = YAML::LoadFile(config_path);
      YAML::Node lidar_config = config["lidar"];

      if (!lidar_config || lidar_config.size() == 0) {
        RCLCPP_ERROR(get_logger(), "No lidar entries found in config file: %s", config_path.c_str());
        return;
      }

      // One component instance = one lidar. For multi-lidar, load multiple component instances.
      driver_ = std::make_shared<SourceDriver>(SourceType::DATA_FROM_LIDAR);
      driver_->Init(lidar_config[0], shared_from_this());
      driver_->Start();
      RCLCPP_INFO(get_logger(), "Hesai driver component started");
    });
  }

  ~HesaiDriverComponent() override {
    if (driver_) {
      driver_->Stop();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr init_timer_;
  SourceDriver::Ptr driver_;
};

}  // namespace hesai_ros_driver

RCLCPP_COMPONENTS_REGISTER_NODE(hesai_ros_driver::HesaiDriverComponent)
