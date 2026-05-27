// SPDX-License-Identifier: BSD-3-Clause
#ifndef _DS3D_DATALOADER_RS_LIDAR_SOURCE_IMPL_H
#define _DS3D_DATALOADER_RS_LIDAR_SOURCE_IMPL_H

#include "rs_lidar_source.h"
#include "rs_lidar_config.h"
#include "ds3d/common/impl/impl_frames.h"
#include "ds3d/common/helper/memdata.h"
#include "ds3d/common/helper/cuda_utils.h"

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>

#include <atomic>
#include <memory>

namespace ds3d { namespace impl { namespace rslidar {

// rs_driver puts PointXYZI and the PointCloudT template at the GLOBAL
// namespace; the driver / SyncQueue / Error live in robosense::lidar. Bridge here.
using PointT       = ::PointXYZI;
using RsCloud      = ::PointCloudT<PointT>;
using MemPtr       = std::unique_ptr<MemData>;

class RsLidarSourceImpl : public SyncImplDataLoader {
public:
    RsLidarSourceImpl();
    ~RsLidarSourceImpl() override;

protected:
    ErrCode startImpl(const std::string& content, const std::string& path) override;
    ErrCode readDataImpl(GuardDataMap& datamap) override;
    ErrCode flushImpl() final { return ErrCode::kGood; }
    ErrCode stopImpl() override;

private:
    ErrCode reserveMem(Ptr<BufferPool<MemPtr>>& pool, size_t bytes, uint32_t count, const std::string& tag);

    // rs_driver callbacks
    std::shared_ptr<RsCloud> getFreeCloud();
    void putStuffedCloud(std::shared_ptr<RsCloud> msg);
    static void onDriverError(const robosense::lidar::Error& code);

    Config _config;
    std::unique_ptr<robosense::lidar::LidarDriver<RsCloud>> _driver;

    robosense::lidar::SyncQueue<std::shared_ptr<RsCloud>> _freeQueue;
    robosense::lidar::SyncQueue<std::shared_ptr<RsCloud>> _stuffedQueue;

    Ptr<BufferPool<MemPtr>> _bufPool;   // pool of fixed-size output buffers (CPU or GPU)
    Ptr<MemData>            _cpuSwapBuf; // staging for GPU copies
    size_t                  _bytesPerFrame = 0; // = maxPoints * 4 * sizeof(float)

    std::atomic<bool>   _running{false};
    std::atomic<bool>   _firstFrame{true};
    uint64_t            _frameCount = 0;
};

}}}
#endif
