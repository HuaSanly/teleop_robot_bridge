#include "video/cuda_eye_image_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>

#include <EGL/egl.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cudaEGL.h>

#include "nvbufsurface.h"

namespace trb::video
{
namespace
{
struct MappedEglFrame
{
    NvBufSurface *surface = nullptr;
    int fd = -1;
    CUgraphicsResource resource = nullptr;
    CUeglFrame frame{};
    bool valid = false;
};

void setFailure(CudaEyeImageResult *result, const char *stage, int error_code)
{
    if (!result)
    {
        return;
    }
    result->ok = false;
    result->error_stage = stage;
    result->error_code = error_code;
}

bool ensureCudaContext(bool &initialized, CudaEyeImageResult *result)
{
    if (initialized)
    {
        return true;
    }
    const cudaError_t ret = cudaFree(nullptr);
    if (ret != cudaSuccess)
    {
        setFailure(result, "init-cuda", static_cast<int>(ret));
        return false;
    }
    initialized = true;
    return true;
}

void resetMappedFrame(MappedEglFrame &mapped)
{
    if (mapped.resource)
    {
        (void)cuGraphicsUnregisterResource(mapped.resource);
    }
    if (mapped.surface)
    {
        (void)NvBufSurfaceUnMapEglImage(mapped.surface, 0);
    }
    mapped = MappedEglFrame{};
}

bool mapEglFrame(NvBufSurface *surface, MappedEglFrame &mapped, const char *stage, CudaEyeImageResult *result)
{
    if (!surface || surface->numFilled == 0)
    {
        setFailure(result, stage, -1);
        return false;
    }

    const int fd = surface->surfaceList[0].bufferDesc;
    if (mapped.valid && mapped.surface == surface && mapped.fd == fd)
    {
        return true;
    }

    resetMappedFrame(mapped);
    if (NvBufSurfaceMapEglImage(surface, 0) != 0)
    {
        setFailure(result, stage, -2);
        return false;
    }

    const EGLImageKHR egl_image = static_cast<EGLImageKHR>(surface->surfaceList[0].mappedAddr.eglImage);
    if (!egl_image)
    {
        (void)NvBufSurfaceUnMapEglImage(surface, 0);
        setFailure(result, stage, -3);
        return false;
    }

    CUgraphicsResource resource = nullptr;
    CUresult cu_ret = cuGraphicsEGLRegisterImage(&resource, egl_image, CU_GRAPHICS_MAP_RESOURCE_FLAGS_NONE);
    if (cu_ret != CUDA_SUCCESS)
    {
        (void)NvBufSurfaceUnMapEglImage(surface, 0);
        setFailure(result, stage, static_cast<int>(cu_ret));
        return false;
    }

    CUeglFrame frame{};
    cu_ret = cuGraphicsResourceGetMappedEglFrame(&frame, resource, 0, 0);
    if (cu_ret != CUDA_SUCCESS)
    {
        (void)cuGraphicsUnregisterResource(resource);
        (void)NvBufSurfaceUnMapEglImage(surface, 0);
        setFailure(result, stage, static_cast<int>(cu_ret));
        return false;
    }

    mapped.surface = surface;
    mapped.fd = fd;
    mapped.resource = resource;
    mapped.frame = frame;
    mapped.valid = true;
    return true;
}

__device__ __forceinline__ float clampFloat(float value, float lo, float hi)
{
    return fminf(fmaxf(value, lo), hi);
}

__device__ __forceinline__ uint8_t sampleU8(const uint8_t *plane, int pitch, int width, int height, float sx, float sy)
{
    sx = clampFloat(sx, 0.0f, static_cast<float>(width - 1));
    sy = clampFloat(sy, 0.0f, static_cast<float>(height - 1));
    const int x0 = static_cast<int>(floorf(sx));
    const int y0 = static_cast<int>(floorf(sy));
    const int x1 = min(x0 + 1, width - 1);
    const int y1 = min(y0 + 1, height - 1);
    const float fx = sx - static_cast<float>(x0);
    const float fy = sy - static_cast<float>(y0);

    const float p00 = static_cast<float>(plane[y0 * pitch + x0]);
    const float p10 = static_cast<float>(plane[y0 * pitch + x1]);
    const float p01 = static_cast<float>(plane[y1 * pitch + x0]);
    const float p11 = static_cast<float>(plane[y1 * pitch + x1]);
    const float top = p00 + (p10 - p00) * fx;
    const float bot = p01 + (p11 - p01) * fx;
    const float value = top + (bot - top) * fy;
    return static_cast<uint8_t>(clampFloat(value + 0.5f, 0.0f, 255.0f));
}

__device__ __forceinline__ void nv12ToRgb(uint8_t y, uint8_t u, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b)
{
    const int yy = static_cast<int>(y);
    const int uu = static_cast<int>(u) - 128;
    const int vv = static_cast<int>(v) - 128;
    int rr = yy + ((359 * vv) >> 8);
    int gg = yy - ((88 * uu + 183 * vv) >> 8);
    int bb = yy + ((454 * uu) >> 8);
    rr = rr < 0 ? 0 : (rr > 255 ? 255 : rr);
    gg = gg < 0 ? 0 : (gg > 255 ? 255 : gg);
    bb = bb < 0 ? 0 : (bb > 255 ? 255 : bb);
    r = static_cast<uint8_t>(rr);
    g = static_cast<uint8_t>(gg);
    b = static_cast<uint8_t>(bb);
}

__global__ void eyeRgbKernel(const uint8_t *src_y,
                             const uint8_t *src_uv,
                             uint8_t *dst_rgb,
                             int src_width,
                             int src_height,
                             int src_y_pitch,
                             int src_uv_pitch,
                             int out_width,
                             int out_height,
                             int eye_offset_x)
{
    const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
    if (x >= out_width || y >= out_height)
    {
        return;
    }

    const float sx = static_cast<float>(eye_offset_x) +
                     (static_cast<float>(x) + 0.5f) * static_cast<float>(src_width / 2) / static_cast<float>(out_width) - 0.5f;
    const float sy = (static_cast<float>(y) + 0.5f) * static_cast<float>(src_height) / static_cast<float>(out_height) - 0.5f;

    const uint8_t yv = sampleU8(src_y, src_y_pitch, src_width, src_height, sx, sy);
    const float uv_x = sx * 0.5f;
    const float uv_y = sy * 0.5f;
    const int uv_width = src_width / 2;
    const int uv_height = src_height / 2;
    const int x0 = static_cast<int>(floorf(clampFloat(uv_x, 0.0f, static_cast<float>(uv_width - 1))));
    const int y0 = static_cast<int>(floorf(clampFloat(uv_y, 0.0f, static_cast<float>(uv_height - 1))));
    const int x1 = min(x0 + 1, uv_width - 1);
    const int y1 = min(y0 + 1, uv_height - 1);
    const float fx = clampFloat(uv_x, 0.0f, static_cast<float>(uv_width - 1)) - static_cast<float>(x0);
    const float fy = clampFloat(uv_y, 0.0f, static_cast<float>(uv_height - 1)) - static_cast<float>(y0);
    const uint8_t *row0 = src_uv + y0 * src_uv_pitch;
    const uint8_t *row1 = src_uv + y1 * src_uv_pitch;
    const float u00 = row0[x0 * 2];
    const float u10 = row0[x1 * 2];
    const float u01 = row1[x0 * 2];
    const float u11 = row1[x1 * 2];
    const float v00 = row0[x0 * 2 + 1];
    const float v10 = row0[x1 * 2 + 1];
    const float v01 = row1[x0 * 2 + 1];
    const float v11 = row1[x1 * 2 + 1];
    const float u_top = u00 + (u10 - u00) * fx;
    const float u_bot = u01 + (u11 - u01) * fx;
    const float v_top = v00 + (v10 - v00) * fx;
    const float v_bot = v01 + (v11 - v01) * fx;
    const uint8_t u = static_cast<uint8_t>(clampFloat(u_top + (u_bot - u_top) * fy + 0.5f, 0.0f, 255.0f));
    const uint8_t v = static_cast<uint8_t>(clampFloat(v_top + (v_bot - v_top) * fy + 0.5f, 0.0f, 255.0f));

    uint8_t r, g, b;
    nv12ToRgb(yv, u, v, r, g, b);
    const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(out_width) + static_cast<size_t>(x)) * 3u;
    dst_rgb[idx + 0] = r;
    dst_rgb[idx + 1] = g;
    dst_rgb[idx + 2] = b;
}

} // namespace

class CudaEyeImageProcessor::Impl
{
public:
    ~Impl()
    {
        reset();
    }

    bool initialize(uint32_t src_width, uint32_t src_height, uint32_t out_width, uint32_t out_height)
    {
        reset();
        src_width_ = src_width;
        src_height_ = src_height;
        out_width_ = out_width;
        out_height_ = out_height;
        if (src_width_ == 0 || src_height_ == 0 || (src_width_ & 1u) != 0 || out_width_ == 0 || out_height_ == 0)
        {
            return false;
        }

        CudaEyeImageResult result;
        if (!ensureCudaContext(cuda_initialized_, &result))
        {
            return false;
        }

        cudaError_t ret = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
        if (ret != cudaSuccess)
        {
            stream_ = nullptr;
            return false;
        }
        ret = cudaEventCreateWithFlags(&done_event_, cudaEventDisableTiming);
        if (ret != cudaSuccess)
        {
            cudaStreamDestroy(stream_);
            stream_ = nullptr;
            done_event_ = nullptr;
            return false;
        }
        rgb_bytes_ = static_cast<size_t>(out_width_) * static_cast<size_t>(out_height_) * 3u;
        ret = cudaMalloc(reinterpret_cast<void **>(&device_rgb_), rgb_bytes_ * 2u);
        if (ret != cudaSuccess)
        {
            reset();
            return false;
        }
        return true;
    }

    bool processEye(NvBufSurface *src, bool is_right_eye, uint8_t *rgb_out, size_t rgb_size, CudaEyeImageResult *result)
    {
        return processInternal(src,
                               !is_right_eye,
                               is_right_eye,
                               is_right_eye ? nullptr : rgb_out,
                               is_right_eye ? rgb_out : nullptr,
                               rgb_size,
                               result);
    }

    bool processEyes(NvBufSurface *src,
                     uint8_t *left_rgb_out,
                     uint8_t *right_rgb_out,
                     size_t rgb_size,
                     CudaEyeImageResult *result)
    {
        return processInternal(src, true, true, left_rgb_out, right_rgb_out, rgb_size, result);
    }

    bool processInternal(NvBufSurface *src,
                         bool process_left,
                         bool process_right,
                         uint8_t *left_rgb_out,
                         uint8_t *right_rgb_out,
                         size_t rgb_size,
                         CudaEyeImageResult *result)
    {
        if (result)
        {
            *result = CudaEyeImageResult{};
        }
        if (!src || (!process_left && !process_right) || !stream_ || !device_rgb_ ||
            (process_left && (!left_rgb_out || rgb_size < rgb_bytes_)) ||
            (process_right && (!right_rgb_out || rgb_size < rgb_bytes_)))
        {
            setFailure(result, "invalid-input", -1);
            return false;
        }

        if (!ensureCudaContext(cuda_initialized_, result))
        {
            return false;
        }

        MappedEglFrame src_frame;
        const auto map_start = std::chrono::steady_clock::now();
        if (!mapEglFrame(src, src_frame, "map-src-egl", result))
        {
            return false;
        }
        const auto map_end = std::chrono::steady_clock::now();

        if (src_frame.frame.frameType != CU_EGL_FRAME_TYPE_PITCH ||
            src_frame.frame.planeCount < 2 ||
            !src_frame.frame.frame.pPitch[0] ||
            !src_frame.frame.frame.pPitch[1])
        {
            setFailure(result, "validate-egl-frame", -2);
            resetMappedFrame(src_frame);
            return false;
        }

        const auto &sp = src->surfaceList[0];
        const auto &pp = sp.planeParams;
        const int eye_offset_x = static_cast<int>(src_width_ / 2u);

        const dim3 block(16, 16);
        const dim3 grid((out_width_ + block.x - 1) / block.x,
                        (out_height_ + block.y - 1) / block.y);

        const auto kernel_start = std::chrono::steady_clock::now();
        const auto *src_y = static_cast<const uint8_t *>(src_frame.frame.frame.pPitch[0]);
        const auto *src_uv = static_cast<const uint8_t *>(src_frame.frame.frame.pPitch[1]);
        if (process_left)
        {
            eyeRgbKernel<<<grid, block, 0, stream_>>>(
                src_y,
                src_uv,
                device_rgb_,
                static_cast<int>(src_width_),
                static_cast<int>(src_height_),
                static_cast<int>(pp.pitch[0]),
                static_cast<int>(pp.pitch[1]),
                static_cast<int>(out_width_),
                static_cast<int>(out_height_),
                0);
        }
        if (process_right)
        {
            eyeRgbKernel<<<grid, block, 0, stream_>>>(
                src_y,
                src_uv,
                device_rgb_ + rgb_bytes_,
                static_cast<int>(src_width_),
                static_cast<int>(src_height_),
                static_cast<int>(pp.pitch[0]),
                static_cast<int>(pp.pitch[1]),
                static_cast<int>(out_width_),
                static_cast<int>(out_height_),
                eye_offset_x);
        }

        cudaError_t ret = cudaGetLastError();
        if (ret != cudaSuccess)
        {
            setFailure(result, "launch-kernel", static_cast<int>(ret));
            resetMappedFrame(src_frame);
            return false;
        }
        if (process_left)
        {
            ret = cudaMemcpyAsync(left_rgb_out, device_rgb_, rgb_bytes_, cudaMemcpyDeviceToHost, stream_);
            if (ret != cudaSuccess)
            {
                setFailure(result, "copy-left-rgb", static_cast<int>(ret));
                resetMappedFrame(src_frame);
                return false;
            }
        }
        if (process_right)
        {
            ret = cudaMemcpyAsync(right_rgb_out, device_rgb_ + rgb_bytes_, rgb_bytes_, cudaMemcpyDeviceToHost, stream_);
            if (ret != cudaSuccess)
            {
                setFailure(result, "copy-right-rgb", static_cast<int>(ret));
                resetMappedFrame(src_frame);
                return false;
            }
        }

        const auto sync_start = std::chrono::steady_clock::now();
        ret = cudaEventRecord(done_event_, stream_);
        if (ret == cudaSuccess)
        {
            ret = cudaEventSynchronize(done_event_);
        }
        const auto sync_end = std::chrono::steady_clock::now();
        resetMappedFrame(src_frame);

        if (result)
        {
            result->map_us = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::microseconds>(map_end - map_start).count());
            result->kernel_us = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::microseconds>(sync_start - kernel_start).count());
            result->sync_us = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::microseconds>(sync_end - sync_start).count());
        }
        if (ret != cudaSuccess)
        {
            setFailure(result, "sync-kernel", static_cast<int>(ret));
            return false;
        }
        if (result)
        {
            result->ok = true;
        }
        return true;
    }

    void reset()
    {
        if (device_rgb_)
        {
            cudaFree(device_rgb_);
            device_rgb_ = nullptr;
        }
        if (done_event_)
        {
            cudaEventDestroy(done_event_);
            done_event_ = nullptr;
        }
        if (stream_)
        {
            cudaStreamDestroy(stream_);
            stream_ = nullptr;
        }
        cuda_initialized_ = false;
        src_width_ = 0;
        src_height_ = 0;
        out_width_ = 0;
        out_height_ = 0;
        rgb_bytes_ = 0;
    }

private:
    bool cuda_initialized_ = false;
    uint32_t src_width_ = 0;
    uint32_t src_height_ = 0;
    uint32_t out_width_ = 0;
    uint32_t out_height_ = 0;
    uint8_t *device_rgb_ = nullptr;
    size_t rgb_bytes_ = 0;
    cudaStream_t stream_ = nullptr;
    cudaEvent_t done_event_ = nullptr;
};

CudaEyeImageProcessor::CudaEyeImageProcessor()
    : impl_(std::make_unique<Impl>())
{
}

CudaEyeImageProcessor::~CudaEyeImageProcessor() = default;

bool CudaEyeImageProcessor::initialize(uint32_t src_width, uint32_t src_height, uint32_t out_width, uint32_t out_height)
{
    return impl_->initialize(src_width, src_height, out_width, out_height);
}

bool CudaEyeImageProcessor::processEye(NvBufSurface *src, bool is_right_eye, uint8_t *rgb_out, size_t rgb_size, CudaEyeImageResult *result)
{
    return impl_->processEye(src, is_right_eye, rgb_out, rgb_size, result);
}

bool CudaEyeImageProcessor::processEyes(NvBufSurface *src,
                                        uint8_t *left_rgb_out,
                                        uint8_t *right_rgb_out,
                                        size_t rgb_size,
                                        CudaEyeImageResult *result)
{
    return impl_->processEyes(src, left_rgb_out, right_rgb_out, rgb_size, result);
}

void CudaEyeImageProcessor::reset()
{
    impl_->reset();
}

} // namespace trb::video
