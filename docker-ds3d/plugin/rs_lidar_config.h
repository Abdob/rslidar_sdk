// SPDX-License-Identifier: BSD-3-Clause
#ifndef _DS3D_DATALOADER_RS_LIDAR_CONFIG_H
#define _DS3D_DATALOADER_RS_LIDAR_CONFIG_H

#include <ds3d/common/common.h>
#include <ds3d/common/func_utils.h>
#include "ds3d/common/hpp/yaml_config.hpp"
#include "ds3d/common/idatatype.h"

#include <rs_driver/driver/driver_param.hpp>

namespace ds3d { namespace impl { namespace rslidar {

struct Config {
    config::ComponentConfig compConfig;

    // rs_driver
    std::string lidarTypeStr  = "RSAIRY";
    uint16_t    msopPort      = 6699;
    uint16_t    difopPort     = 7788;
    uint16_t    imuPort       = 0;       // 0 = disabled
    std::string hostAddress   = "0.0.0.0";
    std::string groupAddress  = "0.0.0.0";
    float       minDistance   = 0.2f;
    float       maxDistance   = 200.0f;
    bool        denseMode     = false;
    bool        useLidarClock = true;

    // Ds3D
    MemType                  memType    = MemType::kGpuCuda;
    int                      gpuId      = 0;
    uint32_t                 maxPoints  = 102400; // upper bound; AIRY frame is ~86k
    uint32_t                 memPoolSize = 4;
    std::vector<std::string> datamapKey {"DS3D::LidarXYZI"};
    uint32_t                 sourceId   = 0;
    bool                     fixedPointsNum = false; // pad with zeros up to maxPoints, but report actual N
    uint32_t                 queueTimeoutMs = 2000;
};

inline robosense::lidar::LidarType parseLidarType(const std::string& s) {
    using LT = robosense::lidar::LidarType;
    // Mechanical
    if (s == "RS16")          return LT::RS16;
    if (s == "RS32")          return LT::RS32;
    if (s == "RSBP")          return LT::RSBP;
    if (s == "RSHELIOS")      return LT::RSHELIOS;
    if (s == "RSHELIOS_16P")  return LT::RSHELIOS_16P;
    if (s == "RS128")         return LT::RS128;
    if (s == "RS80")          return LT::RS80;
    if (s == "RS48")          return LT::RS48;
    if (s == "RSP128")        return LT::RSP128;
    if (s == "RSP80")         return LT::RSP80;
    if (s == "RSP48")         return LT::RSP48;
    // MEMS / solid state
    if (s == "RSM1")          return LT::RSM1;
    if (s == "RSM1_JUMBO")    return LT::RSM1_JUMBO;
    if (s == "RSM2")          return LT::RSM2;
    if (s == "RSM3")          return LT::RSM3;
    if (s == "RSE1")          return LT::RSE1;
    if (s == "RSMX")          return LT::RSMX;
    if (s == "RSEMX")         return LT::RSEMX;
    if (s == "RSAIRY")        return LT::RSAIRY;
    if (s == "RSFAIRY")       return LT::RSFAIRY;
    LOG_WARNING("Unknown lidar_type '%s', defaulting to RSAIRY", s.c_str());
    return LT::RSAIRY;
}

inline ErrCode parseConfig(const std::string& content, const std::string& path, Config& cfg) {
    DS3D_ERROR_RETURN(
        config::parseComponentConfig(content, path, cfg.compConfig),
        "parse rslidar dataloader component content failed");

    YAML::Node node = YAML::Load(cfg.compConfig.configBody);

    if (node["lidar_type"])     cfg.lidarTypeStr  = node["lidar_type"].as<std::string>();
    if (node["msop_port"])      cfg.msopPort      = node["msop_port"].as<uint16_t>();
    if (node["difop_port"])     cfg.difopPort     = node["difop_port"].as<uint16_t>();
    if (node["imu_port"])       cfg.imuPort       = node["imu_port"].as<uint16_t>();
    if (node["host_address"])   cfg.hostAddress   = node["host_address"].as<std::string>();
    if (node["group_address"])  cfg.groupAddress  = node["group_address"].as<std::string>();
    if (node["min_distance"])   cfg.minDistance   = node["min_distance"].as<float>();
    if (node["max_distance"])   cfg.maxDistance   = node["max_distance"].as<float>();
    if (node["dense_points"])   cfg.denseMode     = node["dense_points"].as<bool>();
    if (node["use_lidar_clock"]) cfg.useLidarClock = node["use_lidar_clock"].as<bool>();

    if (node["mem_type"]) {
        auto s = node["mem_type"].as<std::string>();
        if (strncasecmp(s.c_str(), "cpu", s.size()) == 0) {
            cfg.memType = MemType::kCpu;
        } else if (strncasecmp(s.c_str(), "gpu", s.size()) == 0) {
            cfg.memType = MemType::kGpuCuda;
        } else {
            LOG_WARNING("unknown mem_type '%s'", s.c_str());
        }
    }
    if (node["gpu_id"])         cfg.gpuId       = node["gpu_id"].as<int>();
    if (node["max_points"])     cfg.maxPoints   = node["max_points"].as<uint32_t>();
    if (node["points_num"])     cfg.maxPoints   = node["points_num"].as<uint32_t>(); // alias for compat
    if (node["mem_pool_size"])  cfg.memPoolSize = node["mem_pool_size"].as<uint32_t>();
    if (node["source_id"])      cfg.sourceId    = node["source_id"].as<uint32_t>();
    if (node["fixed_points_num"]) cfg.fixedPointsNum = node["fixed_points_num"].as<bool>();
    if (node["queue_timeout_ms"]) cfg.queueTimeoutMs = node["queue_timeout_ms"].as<uint32_t>();
    if (node["output_datamap_key"]) {
        auto k = node["output_datamap_key"];
        if (k.IsSequence()) cfg.datamapKey = k.as<std::vector<std::string>>();
        else { cfg.datamapKey.resize(1); cfg.datamapKey[0] = k.as<std::string>(); }
    }
    return ErrCode::kGood;
}

}}}

#endif
