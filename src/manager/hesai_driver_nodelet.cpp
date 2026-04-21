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
 * File: hesai_driver_nodelet.cpp
 * Description: ROS1 nodelet wrapper for the Hesai LiDAR driver.
 *              Allows the driver to be loaded into a nodelet manager
 *              alongside other nodelets (e.g. filter, assembler) for
 *              in-process communication.
 */

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

#include "source_driver_ros1.hpp"

namespace hesai_ros_driver {

class HesaiDriverNodelet : public nodelet::Nodelet {
public:
  ~HesaiDriverNodelet() override {
    if (driver_) {
      driver_->Stop();
    }
  }

private:
  void onInit() override {
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    std::string config_path;
    pnh.param("config_path", config_path, std::string(""));
    if (config_path.empty()) {
      config_path = ros::package::getPath("hesai_ros_driver")
                    + "/config/config.yaml";
    }

    ROS_INFO("Loading Hesai config from: %s", config_path.c_str());
    YAML::Node config = YAML::LoadFile(config_path);
    YAML::Node lidar_config = config["lidar"];

    if (!lidar_config || lidar_config.size() == 0) {
      ROS_ERROR("No lidar entries found in config file: %s", config_path.c_str());
      return;
    }

    // One nodelet instance = one lidar. For multi-lidar, load multiple nodelet instances.
    auto nh = std::make_shared<ros::NodeHandle>(getNodeHandle());
    driver_ = std::make_shared<SourceDriver>(SourceType::DATA_FROM_LIDAR);
    driver_->Init(lidar_config[0], nh);
    driver_->Start();
    ROS_INFO("Hesai driver nodelet started");
  }

  SourceDriver::Ptr driver_;
};

}  // namespace hesai_ros_driver

PLUGINLIB_EXPORT_CLASS(hesai_ros_driver::HesaiDriverNodelet, nodelet::Nodelet)
