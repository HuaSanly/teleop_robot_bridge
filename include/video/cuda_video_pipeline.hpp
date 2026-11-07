#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace trb::video
{

struct CudaPipelineStats
{
    uint64_t submitted = 0;
    uint64_t completed = 0;
    uint64_t failures = 0;
    uint64_t kernel_us_total = 0;
    uint64_t device_to_host_us_total = 0;
};

// CUDA implementation details are intentionally hidden from the ROS/video
// classes.  The implementation uses Jetson NvBufSurface EGL interop and is
// compiled only when the Jetson CUDA toolchain is available.
class CudaStereoRemapper
{
public:
    CudaStereoRemapper();
    ~CudaStereoRemapper();

    CudaStereoRemapper(const CudaStereoRemapper &) = delete;
    CudaStereoRemapper &operator=(const CudaStereoRemapper &) = delete;

    bool initialize(uint32_t width, uint32_t height,
                    const std::vector<float> &map_xy);
    bool process(int input_fd, int output_fd);
    bool isInitialized() const;
    CudaPipelineStats consumeStats();

private:
    void *impl_{nullptr};
};

class CudaEyeProcessor
{
public:
    CudaEyeProcessor();
    ~CudaEyeProcessor();

    CudaEyeProcessor(const CudaEyeProcessor &) = delete;
    CudaEyeProcessor &operator=(const CudaEyeProcessor &) = delete;

    bool initialize(uint32_t src_width, uint32_t src_height,
                    uint32_t output_width, uint32_t output_height,
                    uint32_t slot_count = 3);

    // Runs crop/scale/NV12->RGB on CUDA and waits only on this worker's CUDA
    // stream before returning. The caller owns the returned slot until the
    // corresponding ROS publish calls have copied its data.
    bool process(int input_fd, uint32_t slot);

    uint32_t slotCount() const;
    const uint8_t *leftData(uint32_t slot) const;
    const uint8_t *rightData(uint32_t slot) const;
    size_t rgbBytes() const;
    bool isInitialized() const;
    CudaPipelineStats consumeStats();

private:
    void *impl_{nullptr};
};

} // namespace trb::video
