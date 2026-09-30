#pragma once
#include <stdexcept>
#include "utility/yaml_reader.hpp"
#include "time_sync_policy.hpp"
#ifdef __CUDACC__
  #include "hesai_lidar_sdk_gpu.cuh"
#else
  #include "hesai_lidar_sdk.hpp"
#endif
#ifdef ROS_FOUND
  #include <ros/package.h>
#endif
class DriveYamlParam
{
public:
    DriveYamlParam() {};
    ~DriveYamlParam() {};

    // Resolve a path relative to the package root if it is not absolute
    static std::string ResolvePath(const std::string &path) {
        if (path.empty() || path[0] == '/') return path;
        std::string pkg_root;
#ifdef RUN_IN_ROS_WORKSPACE
        pkg_root = ros::package::getPath("hesai_ros_driver");
#else
        pkg_root = (std::string)PROJECT_PATH;
#endif
        return pkg_root + "/" + path;
    }

    bool GetDriveYamlParam(const YAML::Node& config, DriverParam &driver_param)
    {
        YAML::Node driver_config = YamlSubNodeAbort(config, "driver");
        int source_type;

        // input related
        YamlRead<uint16_t>(   driver_config, "udp_port",                driver_param.input_param.udp_port, 2368);
        YamlRead<uint16_t>(   driver_config, "ptc_port",                driver_param.input_param.ptc_port, 9347);
        YamlRead<std::string>(driver_config, "host_ip_address",         driver_param.input_param.host_ip_address, "192.168.1.100");
        YamlRead<std::string>(driver_config, "group_address",           driver_param.input_param.multicast_ip_address, "");
        YamlRead<std::string>(driver_config, "pcap_path",               driver_param.input_param.pcap_path, "");
        YamlRead<std::string>(driver_config, "firetimes_path",          driver_param.input_param.firetimes_path, "");
        YamlRead<std::string>(driver_config, "correction_file_path",    driver_param.input_param.correction_file_path, "");
        // Resolve relative paths against the package root
        driver_param.input_param.firetimes_path = ResolvePath(driver_param.input_param.firetimes_path);
        driver_param.input_param.correction_file_path = ResolvePath(driver_param.input_param.correction_file_path);
        driver_param.input_param.pcap_path = ResolvePath(driver_param.input_param.pcap_path);
        YamlRead<int>(        driver_config, "standby_mode",            driver_param.input_param.standby_mode, -1);
        YamlRead<int>(        driver_config, "speed",                   driver_param.input_param.speed, -1);
        // decoder related
        YamlRead<bool>(       driver_config, "pcap_play_synchronization", driver_param.decoder_param.pcap_play_synchronization, false);
        YamlRead<float>(      driver_config, "x",                         driver_param.decoder_param.transform_param.x, 0);
        YamlRead<float>(      driver_config, "y",                         driver_param.decoder_param.transform_param.y, 0);
        YamlRead<float>(      driver_config, "z",                         driver_param.decoder_param.transform_param.z, 0);
        YamlRead<float>(      driver_config, "roll",                      driver_param.decoder_param.transform_param.roll, 0);
        YamlRead<float>(      driver_config, "pitch",                     driver_param.decoder_param.transform_param.pitch, 0);
        YamlRead<float>(      driver_config, "yaw",                       driver_param.decoder_param.transform_param.yaw, 0);
        YamlRead<std::string>(driver_config, "device_ip_address",         driver_param.input_param.device_ip_address, "192.168.1.201");
        YamlRead<float>(      driver_config, "frame_start_azimuth",       driver_param.decoder_param.frame_start_azimuth, -1);
        YamlRead<uint16_t>(   driver_config, "use_timestamp_type",        driver_param.decoder_param.use_timestamp_type, 0);
        if (driver_param.decoder_param.use_timestamp_type > 1) {
            std::cerr << "Invalid use_timestamp_type="
                      << driver_param.decoder_param.use_timestamp_type
                      << ", defaulting to 0 (sensor time)" << std::endl;
            driver_param.decoder_param.use_timestamp_type = 0;
        }
        YamlRead<int>(        driver_config, "fov_start",                 driver_param.decoder_param.fov_start, -1);
        YamlRead<int>(        driver_config, "fov_end",                   driver_param.decoder_param.fov_end, -1);
        YamlRead<int>(        driver_config, "source_type",               source_type, 0);
        driver_param.input_param.source_type = SourceType(source_type);
        // ROS related
        YamlRead<bool>(       driver_config, "enable_packet_loss_tool",    driver_param.decoder_param.enable_packet_loss_tool, false);
        YamlRead<bool>(       config["ros"], "send_packet_ros",            driver_param.input_param.send_packet_ros, false);
        YamlRead<bool>(       config["ros"], "send_point_cloud_ros",       driver_param.input_param.send_point_cloud_ros, false);
        YamlRead<std::string>(config["ros"], "ros_frame_id",               driver_param.input_param.frame_id, "hesai_lidar");
        YamlRead<std::string>(config["ros"], "ros_send_packet_topic",      driver_param.input_param.ros_send_packet_topic, "hesai_packets");
        YamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", driver_param.input_param.ros_send_point_topic, "hesai_points");
        YamlRead<std::string>(config["ros"], "ros_recv_packet_topic",      driver_param.input_param.ros_recv_packet_topic, "hesai_packets");
        YamlRead<std::string>(config["ros"], "ros_send_packet_loss_topic", driver_param.input_param.ros_send_packet_loss_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_send_ptp_topic",         driver_param.input_param.ros_send_ptp_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_send_correction_topic",  driver_param.input_param.ros_send_correction_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_send_firetime_topic",    driver_param.input_param.ros_send_firetime_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_recv_correction_topic",  driver_param.input_param.ros_recv_correction_topic, NULL_TOPIC);
        return true;
    }

    // Stamping policy (time_sync_policy.hpp). Keys live under `ros:`.
    static void GetTimeSyncParam(const YAML::Node& config, hesai_ros_driver::TimeSyncConfig &cfg)
    {
        const YAML::Node ros_config = config["ros"];

        std::string source;
        YamlRead<std::string>(ros_config, "timestamp_source", source, "auto");
        if (source == "auto") {
            cfg.source = hesai_ros_driver::TimestampSource::kAuto;
        } else if (source == "sensor") {
            cfg.source = hesai_ros_driver::TimestampSource::kSensor;
        } else if (source == "host") {
            cfg.source = hesai_ros_driver::TimestampSource::kHost;
        } else {
            std::cerr << "Invalid timestamp_source=" << source << ", using auto" << std::endl;
            cfg.source = hesai_ros_driver::TimestampSource::kAuto;
        }

        std::string tai;
        YamlRead<std::string>(ros_config, "tai_utc_offset_s", tai, "auto");
        cfg.tai_offset_auto = true;
        if (tai != "auto") {
            try {
                size_t used = 0;
                const int value = std::stoi(tai, &used);
                if (used != tai.size()) throw std::invalid_argument(tai);
                cfg.tai_offset_auto = false;
                cfg.tai_offset_s = value;
            } catch (const std::exception&) {
                std::cerr << "Invalid tai_utc_offset_s=" << tai << ", using auto" << std::endl;
            }
        }

        long long max_offset_ns = cfg.ptp_max_offset_ns;
        YamlRead<long long>(ros_config, "ptp_max_offset_ns", max_offset_ns, max_offset_ns);
        cfg.ptp_max_offset_ns = max_offset_ns;
        YamlRead<double>(ros_config, "ptp_status_max_age_s", cfg.ptp_status_max_age_s, cfg.ptp_status_max_age_s);
        YamlRead<double>(ros_config, "frame_span_max_s", cfg.frame_span_max_s, cfg.frame_span_max_s);
        YamlRead<bool>(ros_config, "publish_time_field", cfg.publish_time_field, cfg.publish_time_field);
    }

};