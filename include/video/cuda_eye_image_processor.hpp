#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

struct NvBufSurface;

namespace trb::video
{

struct CudaEyeImageResult
{
    bool ok = false;
    int error_code = 0;
    const char *error_stage = "";
    int64_t map_us = 0;
    int64_t kernel_us = 0;
    int64_t sync_us = 0;
};

class CudaEyeImageProcessor
{
public:
    CudaEyeImageProcessor();
    ~CudaEyeImageProcessor();

    CudaEyeImageProcessor(const CudaEyeImageProcessor &) = delete;
    CudaEyeImageProcessor &operator=(const CudaEyeImageProcessor &) = delete;

    bool initialize(uint32_t src_width, uint32_t src_height, uint32_t out_width, uint32_t out_height);
    bool processEye(NvBufSurface *src, bool is_right_eye, uint8_t *rgb_out, size_t rgb_size, CudaEyeImageResult *result);
    bool processEyes(NvBufSurface *src,
                     uint8_t *left_rgb_out,
                     uint8_t *right_rgb_out,
                     size_t rgb_size,
                     CudaEyeImageResult *result);
    void reset();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace trb::video
