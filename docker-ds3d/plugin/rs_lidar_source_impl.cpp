// SPDX-License-Identifier: BSD-3-Clause
#include "rs_lidar_source_impl.h"

#include <chrono>
#include <cstring>

namespace ds3d { namespace impl { namespace rslidar {

constexpr uint64_t kNsPerSec  = 1000000000ULL;
constexpr uint64_t kNsPerUsec = 1000ULL;

RsLidarSourceImpl::RsLidarSourceImpl() = default;
RsLidarSourceImpl::~RsLidarSourceImpl() {
    if (_running.load()) { stopImpl(); }
}

void RsLidarSourceImpl::onDriverError(const robosense::lidar::Error& code) {
    // Driver callback runs in its own thread; just log
    LOG_WARNING("rs_driver: %s", code.toString().c_str());
}

std::shared_ptr<RsCloud> RsLidarSourceImpl::getFreeCloud() {
    auto msg = _freeQueue.pop();
    if (msg) return msg;
    return std::make_shared<RsCloud>();
}

void RsLidarSourceImpl::putStuffedCloud(std::shared_ptr<RsCloud> msg) {
    _stuffedQueue.push(msg);
}

ErrCode RsLidarSourceImpl::reserveMem(
    Ptr<BufferPool<MemPtr>>& pool, size_t bytes, uint32_t count, const std::string& tag)
{
    pool = std::make_shared<BufferPool<MemPtr>>(tag);
    for (uint32_t i = 0; i < count; ++i) {
        MemPtr p;
        if (isCpuMem(_config.memType)) {
            p = CpuMemBuf::CreateBuf(bytes);
        } else {
            p = GpuCudaMemBuf::CreateBuf(bytes, _config.gpuId);
        }
        pool->setBuffer(std::move(p));
    }
    DS3D_FAILED_RETURN(
        pool->size() == count, ErrCode::kMem,
        "rs_lidar dataloader: pool '%s' allocation failed", tag.c_str());
    return ErrCode::kGood;
}

ErrCode RsLidarSourceImpl::startImpl(const std::string& content, const std::string& path) {
    LOG_INFO("rs_lidar dataloader starting");

    ErrCode code = config::CatchYamlCall(
        [&, this]() { return parseConfig(content, path, _config); });
    DS3D_ERROR_RETURN(code, "parse rs_lidar config '%s' failed", path.c_str());

    setOutputCaps(_config.compConfig.gstOutCaps);

    _bytesPerFrame = static_cast<size_t>(_config.maxPoints) * 4u * sizeof(float);

    DS3D_ERROR_RETURN(
        reserveMem(_bufPool, _bytesPerFrame, _config.memPoolSize, "rsLidarBufPool"),
        "buffer pool reserve failed");
    if (isGpuMem(_config.memType)) {
        _cpuSwapBuf = CpuMemBuf::CreateBuf(_bytesPerFrame);
        DS_ASSERT(_cpuSwapBuf);
    }

    // Spin up rs_driver
    robosense::lidar::RSDriverParam param;
    param.input_type              = robosense::lidar::InputType::ONLINE_LIDAR;
    param.lidar_type              = parseLidarType(_config.lidarTypeStr);
    param.input_param.msop_port   = _config.msopPort;
    param.input_param.difop_port  = _config.difopPort;
    param.input_param.imu_port    = _config.imuPort;
    param.input_param.host_address  = _config.hostAddress;
    param.input_param.group_address = _config.groupAddress;
    param.decoder_param.min_distance = _config.minDistance;
    param.decoder_param.max_distance = _config.maxDistance;
    param.decoder_param.dense_points = _config.denseMode;
    param.decoder_param.use_lidar_clock = _config.useLidarClock;
    param.print();

    _driver = std::make_unique<robosense::lidar::LidarDriver<RsCloud>>();
    _driver->regPointCloudCallback(
        [this]() { return getFreeCloud(); },
        [this](std::shared_ptr<RsCloud> msg) { putStuffedCloud(std::move(msg)); });
    _driver->regExceptionCallback(&RsLidarSourceImpl::onDriverError);

    DS3D_FAILED_RETURN(
        _driver->init(param), ErrCode::kUnknown, "rs_driver init failed");
    DS3D_FAILED_RETURN(
        _driver->start(), ErrCode::kUnknown, "rs_driver start failed");

    _running.store(true);
    LOG_INFO("rs_lidar dataloader started (lidar=%s msop=%u difop=%u)",
             _config.lidarTypeStr.c_str(), _config.msopPort, _config.difopPort);
    return ErrCode::kGood;
}

ErrCode RsLidarSourceImpl::readDataImpl(GuardDataMap& outData) {
    if (!_running.load()) return ErrCode::kState;

    std::shared_ptr<RsCloud> msg;
    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::milliseconds(_config.queueTimeoutMs);
    while (_running.load()) {
        msg = _stuffedQueue.popWait(50000); // 50 ms in microseconds
        if (msg) break;
        if (std::chrono::steady_clock::now() >= deadline) {
            LOG_WARNING("rs_lidar dataloader: no point cloud within %u ms", _config.queueTimeoutMs);
            return ErrCode::kTimeOut;
        }
    }
    if (!msg) return ErrCode::kState;

    const uint32_t pointsIn = static_cast<uint32_t>(msg->points.size());
    uint32_t pointsToCopy = pointsIn;
    if (pointsToCopy > _config.maxPoints) {
        LOG_DEBUG("rs_lidar: clipping %u points to maxPoints=%u", pointsIn, _config.maxPoints);
        pointsToCopy = _config.maxPoints;
    }

    Ptr<MemData> dstBuf = _bufPool->acquireBuffer();
    Ptr<MemData> cpuBuf = isGpuMem(_config.memType) ? _cpuSwapBuf : dstBuf;

    // rs_driver PointXYZI is packed: 3 floats (x,y,z) + uint8 intensity = 13 bytes.
    // DS3D LidarXYZI expects [N, 4] FP32: convert per-point, intensity -> float in [0,1].
    std::memset(cpuBuf->data, 0, _bytesPerFrame);
    float* dst = static_cast<float*>(cpuBuf->data);
    const PointT* src = msg->points.data();
    for (uint32_t i = 0; i < pointsToCopy; ++i) {
        dst[i * 4 + 0] = src[i].x;
        dst[i * 4 + 1] = src[i].y;
        dst[i * 4 + 2] = src[i].z;
        dst[i * 4 + 3] = static_cast<float>(src[i].intensity) * (1.0f / 255.0f);
    }

    if (isGpuMem(_config.memType)) {
        DS3D_CHECK_CUDA_ERROR(
            cudaSetDevice(_config.gpuId), return ErrCode::kCuda,
            "cudaSetDevice(%d) failed", _config.gpuId);
        DS3D_CHECK_CUDA_ERROR(
            cudaMemcpy(dstBuf->data, cpuBuf->data, _bytesPerFrame, cudaMemcpyHostToDevice),
            return ErrCode::kCuda, "host->device copy failed");
    }

    // Return msg to rs_driver's free pool
    _freeQueue.push(msg);

    uint32_t reportedPoints = _config.fixedPointsNum ? _config.maxPoints : pointsToCopy;

    // Argument-evaluation order is unspecified across the next call; capture
    // the data pointer BEFORE the lambda is allowed to move dstBuf.
    void* const framePtr = dstBuf->data;
    FrameGuard lidarFrame = wrapLidarXYZIFrame<float>(
        framePtr, reportedPoints, _config.memType, 0,
        [keep = std::move(dstBuf)](void*) {});
    DS3D_FAILED_RETURN(
        lidarFrame, ErrCode::kUnknown, "wrapLidarXYZIFrame failed");

    GuardDataMap datamap(NvDs3d_CreateDataHashMap(), true);
    DS3D_FAILED_RETURN(
        datamap.setGuardData(_config.datamapKey[0], lidarFrame) == ErrCode::kGood,
        ErrCode::kUnknown, "datamap setGuardData failed");

    datamap.setData(kSourceId, _config.sourceId);
    if (_firstFrame.exchange(false)) {
        bool first = true;
        datamap.setData(kFirstSourceFrame, first);
    }

    // Timestamp: rs_driver gives seconds-since-epoch as double on the message.
    TimeStamp ts{0};
    ts.t0 = static_cast<uint64_t>(msg->timestamp * static_cast<double>(kNsPerSec));
    datamap.setData(kTimeStamp, ts);

    ++_frameCount;
    if ((_frameCount % 30) == 0) {
        LOG_INFO("rs_lidar: %llu frames forwarded (last=%u pts, ts=%.3f)",
                 static_cast<unsigned long long>(_frameCount),
                 pointsToCopy, msg->timestamp);
    }
    outData = std::move(datamap);
    return ErrCode::kGood;
}

ErrCode RsLidarSourceImpl::stopImpl() {
    LOG_INFO("rs_lidar dataloader stopping (frames=%llu)",
             static_cast<unsigned long long>(_frameCount));
    _running.store(false);
    if (_driver) {
        _driver->stop();
        _driver.reset();
    }
    _bufPool.reset();
    _cpuSwapBuf.reset();
    return ErrCode::kGood;
}

}}}  // ds3d::impl::rslidar

using namespace ds3d;

DS3D_EXTERN_C_BEGIN
DS3D_EXPORT_API abiRefDataLoader* createRsLidarLoader() {
    return NewAbiRef<abiDataLoader>(new impl::rslidar::RsLidarSourceImpl);
}
DS3D_EXTERN_C_END
