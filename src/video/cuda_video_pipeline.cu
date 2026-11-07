#include "video/cuda_video_pipeline.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <EGL/egl.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cudaEGL.h>

#include "nvbufsurface.h"

namespace trb::video
{
namespace
{

bool checkCuda(cudaError_t status, const char *what)
{
    if (status == cudaSuccess)
        return true;
    std::cerr << "[CUDA] " << what << " failed: "
              << cudaGetErrorString(status) << std::endl;
    return false;
}

bool checkCu(CUresult status, const char *what)
{
    if (status == CUDA_SUCCESS)
        return true;
    const char *name = nullptr;
    const char *message = nullptr;
    cuGetErrorName(status, &name);
    cuGetErrorString(status, &message);
    std::cerr << "[CUDA] " << what << " failed: "
              << (name ? name : "unknown") << " ("
              << (message ? message : "unknown") << ")" << std::endl;
    return false;
}

struct RegisteredSurface
{
    NvBufSurface *surface{nullptr};
    EGLImageKHR egl_image{EGL_NO_IMAGE_KHR};
    CUgraphicsResource resource{nullptr};
    CUeglFrame frame{};
    size_t plane_pitch[2]{0, 0};
};

bool registerSurface(int fd, unsigned int flags, RegisteredSurface &out)
{
    if (fd < 0)
        return false;

    if (NvBufSurfaceFromFd(fd, reinterpret_cast<void **>(&out.surface)) != 0 ||
        !out.surface || !out.surface->surfaceList)
    {
        std::cerr << "[CUDA] NvBufSurfaceFromFd failed for fd=" << fd << std::endl;
        return false;
    }

    if (NvBufSurfaceMapEglImage(out.surface, 0) != 0)
    {
        std::cerr << "[CUDA] NvBufSurfaceMapEglImage failed for fd=" << fd << std::endl;
        out.surface = nullptr;
        return false;
    }

    out.egl_image = reinterpret_cast<EGLImageKHR>(
        out.surface->surfaceList[0].mappedAddr.eglImage);
    if (out.egl_image == EGL_NO_IMAGE_KHR)
    {
        NvBufSurfaceUnMapEglImage(out.surface, 0);
        out.surface = nullptr;
        return false;
    }

    if (!checkCu(cuGraphicsEGLRegisterImage(&out.resource, out.egl_image, flags),
                 "cuGraphicsEGLRegisterImage") ||
        !checkCu(cuGraphicsResourceGetMappedEglFrame(&out.frame, out.resource, 0, 0),
                 "cuGraphicsResourceGetMappedEglFrame"))
    {
        if (out.resource)
            cuGraphicsUnregisterResource(out.resource);
        NvBufSurfaceUnMapEglImage(out.surface, 0);
        out = RegisteredSurface{};
        return false;
    }

    if (out.frame.frameType != CU_EGL_FRAME_TYPE_PITCH ||
        out.frame.planeCount < 2 || !out.frame.frame.pPitch[0] ||
        !out.frame.frame.pPitch[1])
    {
        std::cerr << "[CUDA] expected pitch-linear two-plane NV12 EGL frame"
                  << std::endl;
        cuGraphicsUnregisterResource(out.resource);
        NvBufSurfaceUnMapEglImage(out.surface, 0);
        out = RegisteredSurface{};
        return false;
    }
    out.plane_pitch[0] = out.surface->surfaceList[0].planeParams.pitch[0];
    out.plane_pitch[1] = out.surface->surfaceList[0].planeParams.pitch[1];
    if (out.plane_pitch[0] == 0)
        out.plane_pitch[0] = out.frame.pitch;
    if (out.plane_pitch[1] == 0)
        out.plane_pitch[1] = out.frame.pitch;
    return true;
}

void unregisterSurface(RegisteredSurface &surface)
{
    if (surface.resource)
    {
        cuGraphicsUnregisterResource(surface.resource);
        surface.resource = nullptr;
    }
    if (surface.surface)
    {
        NvBufSurfaceUnMapEglImage(surface.surface, 0);
    }
    surface = RegisteredSurface{};
}

__device__ __forceinline__ unsigned char sampleByte(const unsigned char *plane,
                                                     size_t pitch,
                                                     int width,
                                                     int height,
                                                     float x,
                                                     float y)
{
    if (x < 0.0f || y < 0.0f || x > static_cast<float>(width - 1) ||
        y > static_cast<float>(height - 1))
        return 0;

    x = fminf(fmaxf(x, 0.0f), static_cast<float>(width - 1));
    y = fminf(fmaxf(y, 0.0f), static_cast<float>(height - 1));
    const int x0 = static_cast<int>(floorf(x));
    const int y0 = static_cast<int>(floorf(y));
    const int x1 = min(x0 + 1, width - 1);
    const int y1 = min(y0 + 1, height - 1);
    const float ax = x - static_cast<float>(x0);
    const float ay = y - static_cast<float>(y0);

    const float p00 = plane[y0 * pitch + x0];
    const float p01 = plane[y0 * pitch + x1];
    const float p10 = plane[y1 * pitch + x0];
    const float p11 = plane[y1 * pitch + x1];
    return static_cast<unsigned char>(fminf(255.0f,
        fmaxf(0.0f, (p00 + (p01 - p00) * ax) * (1.0f - ay) +
                         (p10 + (p11 - p10) * ax) * ay + 0.5f)));
}

__device__ __forceinline__ uchar2 sampleUv(const unsigned char *plane,
                                            size_t pitch,
                                            int width,
                                            int height,
                                            float x,
                                            float y)
{
    if (x < 0.0f || y < 0.0f || x > static_cast<float>(width - 1) ||
        y > static_cast<float>(height - 1))
        return make_uchar2(128, 128);

    x = fminf(fmaxf(x, 0.0f), static_cast<float>(width - 1));
    y = fminf(fmaxf(y, 0.0f), static_cast<float>(height - 1));
    const int x0 = static_cast<int>(floorf(x));
    const int y0 = static_cast<int>(floorf(y));
    const int x1 = min(x0 + 1, width - 1);
    const int y1 = min(y0 + 1, height - 1);
    const float ax = x - static_cast<float>(x0);
    const float ay = y - static_cast<float>(y0);

    const uchar2 p00 = reinterpret_cast<const uchar2 *>(plane + y0 * pitch)[x0];
    const uchar2 p01 = reinterpret_cast<const uchar2 *>(plane + y0 * pitch)[x1];
    const uchar2 p10 = reinterpret_cast<const uchar2 *>(plane + y1 * pitch)[x0];
    const uchar2 p11 = reinterpret_cast<const uchar2 *>(plane + y1 * pitch)[x1];

    const float u0 = (p00.x + (p01.x - p00.x) * ax) * (1.0f - ay) +
                    (p10.x + (p11.x - p10.x) * ax) * ay;
    const float v0 = (p00.y + (p01.y - p00.y) * ax) * (1.0f - ay) +
                    (p10.y + (p11.y - p10.y) * ax) * ay;
    return make_uchar2(static_cast<unsigned char>(fminf(255.0f, fmaxf(0.0f, u0 + 0.5f))),
                       static_cast<unsigned char>(fminf(255.0f, fmaxf(0.0f, v0 + 0.5f))));
}

__global__ void remapYKernel(const unsigned char *src, size_t src_pitch,
                             unsigned char *dst, size_t dst_pitch,
                             const float2 *map_xy, int width, int height)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height)
        return;
    const float2 p = map_xy[y * width + x];
    dst[y * dst_pitch + x] = sampleByte(src, src_pitch, width, height, p.x, p.y);
}

__global__ void remapUvKernel(const unsigned char *src, size_t src_pitch,
                              unsigned char *dst, size_t dst_pitch,
                              const float2 *map_xy, int width, int height)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    const int uv_width = width / 2;
    const int uv_height = (height + 1) / 2;
    if (x >= uv_width || y >= uv_height)
        return;

    const int map_x = min(2 * x, width - 1);
    const int map_y = min(2 * y, height - 1);
    const float2 p = map_xy[map_y * width + map_x];
    const uchar2 uv = sampleUv(src, src_pitch, uv_width, uv_height,
                               p.x * 0.5f, p.y * 0.5f);
    reinterpret_cast<uchar2 *>(dst + y * dst_pitch)[x] = uv;
}

__global__ void eyeRgbKernel(const unsigned char *src_y, size_t src_y_pitch,
                             const unsigned char *src_uv, size_t src_uv_pitch,
                             unsigned char *left, unsigned char *right,
                             int src_width, int src_height,
                             int output_width, int output_height)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    const int eye = blockIdx.z;
    if (x >= output_width || y >= output_height || eye > 1)
        return;

    const float half_width = static_cast<float>(src_width) * 0.5f;
    const float sx = static_cast<float>(eye) * half_width +
                     ((static_cast<float>(x) + 0.5f) * half_width /
                      static_cast<float>(output_width) - 0.5f);
    const float sy = (static_cast<float>(y) + 0.5f) *
                     static_cast<float>(src_height) /
                     static_cast<float>(output_height) - 0.5f;

    const unsigned char yv = sampleByte(src_y, src_y_pitch, src_width,
                                        src_height, sx, sy);
    const uchar2 uv = sampleUv(src_uv, src_uv_pitch, src_width / 2,
                               (src_height + 1) / 2, sx * 0.5f, sy * 0.5f);
    const int y_i = static_cast<int>(yv);
    const int u = static_cast<int>(uv.x) - 128;
    const int v = static_cast<int>(uv.y) - 128;
    const int r = y_i + ((359 * v) >> 8);
    const int g = y_i - ((88 * u + 183 * v) >> 8);
    const int b = y_i + ((454 * u) >> 8);

    unsigned char *out = eye == 0 ? left : right;
    const size_t offset = (static_cast<size_t>(y) * output_width + x) * 3;
    out[offset + 0] = static_cast<unsigned char>(max(0, min(255, r)));
    out[offset + 1] = static_cast<unsigned char>(max(0, min(255, g)));
    out[offset + 2] = static_cast<unsigned char>(max(0, min(255, b)));
}

struct RemapperImpl
{
    uint32_t width{0};
    uint32_t height{0};
    float2 *map_xy{nullptr};
    cudaStream_t stream{nullptr};
    cudaEvent_t kernel_start{nullptr};
    cudaEvent_t kernel_end{nullptr};
    std::unordered_map<int, RegisteredSurface> surfaces;
    std::atomic<uint64_t> submitted{0};
    std::atomic<uint64_t> completed{0};
    std::atomic<uint64_t> failures{0};
    std::atomic<uint64_t> kernel_us_total{0};

    ~RemapperImpl()
    {
        if (stream)
            cudaStreamSynchronize(stream);
        for (auto &entry : surfaces)
            unregisterSurface(entry.second);
        surfaces.clear();
        if (map_xy)
            cudaFree(map_xy);
        if (kernel_start)
            cudaEventDestroy(kernel_start);
        if (kernel_end)
            cudaEventDestroy(kernel_end);
        if (stream)
            cudaStreamDestroy(stream);
    }

    RegisteredSurface *get(int fd, unsigned int flags)
    {
        auto it = surfaces.find(fd);
        if (it != surfaces.end())
            return &it->second;
        RegisteredSurface surface;
        if (!registerSurface(fd, flags, surface))
            return nullptr;
        auto result = surfaces.emplace(fd, std::move(surface));
        return &result.first->second;
    }
};

struct EyeSlot
{
    unsigned char *device_left{nullptr};
    unsigned char *device_right{nullptr};
    unsigned char *host_left{nullptr};
    unsigned char *host_right{nullptr};
};

struct EyeImpl
{
    uint32_t src_width{0};
    uint32_t src_height{0};
    uint32_t output_width{0};
    uint32_t output_height{0};
    size_t rgb_bytes{0};
    cudaStream_t stream{nullptr};
    cudaEvent_t kernel_start{nullptr};
    cudaEvent_t kernel_end{nullptr};
    cudaEvent_t copy_start{nullptr};
    cudaEvent_t copy_end{nullptr};
    std::vector<EyeSlot> slots;
    std::unordered_map<int, RegisteredSurface> surfaces;
    std::atomic<uint64_t> submitted{0};
    std::atomic<uint64_t> completed{0};
    std::atomic<uint64_t> failures{0};
    std::atomic<uint64_t> kernel_us_total{0};
    std::atomic<uint64_t> device_to_host_us_total{0};

    ~EyeImpl()
    {
        if (stream)
            cudaStreamSynchronize(stream);
        for (auto &entry : surfaces)
            unregisterSurface(entry.second);
        surfaces.clear();
        for (auto &slot : slots)
        {
            if (slot.device_left) cudaFree(slot.device_left);
            if (slot.device_right) cudaFree(slot.device_right);
            if (slot.host_left) cudaFreeHost(slot.host_left);
            if (slot.host_right) cudaFreeHost(slot.host_right);
        }
        if (kernel_start) cudaEventDestroy(kernel_start);
        if (kernel_end) cudaEventDestroy(kernel_end);
        if (copy_start) cudaEventDestroy(copy_start);
        if (copy_end) cudaEventDestroy(copy_end);
        if (stream)
            cudaStreamDestroy(stream);
    }

    RegisteredSurface *get(int fd)
    {
        auto it = surfaces.find(fd);
        if (it != surfaces.end())
            return &it->second;
        RegisteredSurface surface;
        if (!registerSurface(fd, CU_GRAPHICS_MAP_RESOURCE_FLAGS_READ_ONLY, surface))
            return nullptr;
        auto result = surfaces.emplace(fd, std::move(surface));
        return &result.first->second;
    }
};

} // namespace

CudaStereoRemapper::CudaStereoRemapper() = default;

CudaStereoRemapper::~CudaStereoRemapper()
{
    delete static_cast<RemapperImpl *>(impl_);
    impl_ = nullptr;
}

bool CudaStereoRemapper::initialize(uint32_t width, uint32_t height,
                                    const std::vector<float> &map_xy)
{
    delete static_cast<RemapperImpl *>(impl_);
    impl_ = nullptr;
    if (width < 2 || (width & 1u) || height == 0 || map_xy.size() !=
        static_cast<size_t>(width) * height * 2)
        return false;

    if (!checkCuda(cudaFree(0), "cudaFree(0)"))
        return false;
    auto *impl = new RemapperImpl();
    impl->width = width;
    impl->height = height;
    if (!checkCuda(cudaStreamCreateWithFlags(&impl->stream, cudaStreamNonBlocking),
                   "cudaStreamCreateWithFlags") ||
        !checkCuda(cudaEventCreate(&impl->kernel_start),
                   "cudaEventCreate(remap start)") ||
        !checkCuda(cudaEventCreate(&impl->kernel_end),
                   "cudaEventCreate(remap end)") ||
        !checkCuda(cudaMalloc(reinterpret_cast<void **>(&impl->map_xy),
                              map_xy.size() * sizeof(float)),
                   "cudaMalloc(map)"))
    {
        delete impl;
        return false;
    }
    if (!checkCuda(cudaMemcpyAsync(impl->map_xy, map_xy.data(),
                                   map_xy.size() * sizeof(float),
                                   cudaMemcpyHostToDevice, impl->stream),
                   "cudaMemcpyAsync(map)") ||
        !checkCuda(cudaStreamSynchronize(impl->stream), "cudaStreamSynchronize(map)"))
    {
        delete impl;
        return false;
    }
    impl_ = impl;
    return true;
}

bool CudaStereoRemapper::process(int input_fd, int output_fd)
{
    auto *impl = static_cast<RemapperImpl *>(impl_);
    if (!impl)
        return false;
    impl->submitted.fetch_add(1, std::memory_order_relaxed);
    auto fail = [&]() {
        impl->failures.fetch_add(1, std::memory_order_relaxed);
        return false;
    };
    RegisteredSurface *input = impl->get(input_fd, CU_GRAPHICS_MAP_RESOURCE_FLAGS_READ_ONLY);
    RegisteredSurface *output = impl->get(output_fd, CU_GRAPHICS_MAP_RESOURCE_FLAGS_WRITE_DISCARD);
    if (!input || !output)
        return fail();

    const auto *src_y = static_cast<const unsigned char *>(input->frame.frame.pPitch[0]);
    const auto *src_uv = static_cast<const unsigned char *>(input->frame.frame.pPitch[1]);
    auto *dst_y = static_cast<unsigned char *>(output->frame.frame.pPitch[0]);
    auto *dst_uv = static_cast<unsigned char *>(output->frame.frame.pPitch[1]);
    const dim3 block(16, 16);
    const dim3 y_grid((impl->width + block.x - 1) / block.x,
                      (impl->height + block.y - 1) / block.y);
    const dim3 uv_grid(((impl->width / 2) + block.x - 1) / block.x,
                       (((impl->height + 1) / 2) + block.y - 1) / block.y);
    if (!checkCuda(cudaEventRecord(impl->kernel_start, impl->stream),
                   "cudaEventRecord(remap start)"))
        return fail();
    remapYKernel<<<y_grid, block, 0, impl->stream>>>(
        src_y, input->plane_pitch[0], dst_y, output->plane_pitch[0],
        impl->map_xy, static_cast<int>(impl->width), static_cast<int>(impl->height));
    remapUvKernel<<<uv_grid, block, 0, impl->stream>>>(
        src_uv, input->plane_pitch[1], dst_uv, output->plane_pitch[1],
        impl->map_xy, static_cast<int>(impl->width), static_cast<int>(impl->height));
    if (!checkCuda(cudaGetLastError(), "remap kernels") ||
        !checkCuda(cudaEventRecord(impl->kernel_end, impl->stream),
                   "cudaEventRecord(remap end)" ) ||
        !checkCuda(cudaStreamSynchronize(impl->stream), "cudaStreamSynchronize(remap)"))
        return fail();
    float elapsed_ms = 0.0f;
    if (checkCuda(cudaEventElapsedTime(&elapsed_ms, impl->kernel_start, impl->kernel_end),
                  "cudaEventElapsedTime(remap)"))
    {
        impl->kernel_us_total.fetch_add(static_cast<uint64_t>(elapsed_ms * 1000.0f),
                                        std::memory_order_relaxed);
    }
    impl->completed.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool CudaStereoRemapper::isInitialized() const
{
    return impl_ != nullptr;
}

CudaPipelineStats CudaStereoRemapper::consumeStats()
{
    CudaPipelineStats stats;
    auto *impl = static_cast<RemapperImpl *>(impl_);
    if (!impl)
        return stats;
    stats.submitted = impl->submitted.exchange(0, std::memory_order_relaxed);
    stats.completed = impl->completed.exchange(0, std::memory_order_relaxed);
    stats.failures = impl->failures.exchange(0, std::memory_order_relaxed);
    stats.kernel_us_total = impl->kernel_us_total.exchange(0, std::memory_order_relaxed);
    return stats;
}

CudaEyeProcessor::CudaEyeProcessor() = default;

CudaEyeProcessor::~CudaEyeProcessor()
{
    delete static_cast<EyeImpl *>(impl_);
    impl_ = nullptr;
}

bool CudaEyeProcessor::initialize(uint32_t src_width, uint32_t src_height,
                                  uint32_t output_width, uint32_t output_height,
                                  uint32_t slot_count)
{
    delete static_cast<EyeImpl *>(impl_);
    impl_ = nullptr;
    if (src_width < 2 || (src_width & 1u) || src_height == 0 || output_width == 0 ||
        output_height == 0 || slot_count == 0)
        return false;
    if (!checkCuda(cudaFree(0), "cudaFree(0)"))
        return false;

    auto *impl = new EyeImpl();
    impl->src_width = src_width;
    impl->src_height = src_height;
    impl->output_width = output_width;
    impl->output_height = output_height;
    impl->rgb_bytes = static_cast<size_t>(output_width) * output_height * 3;
    if (!checkCuda(cudaStreamCreateWithFlags(&impl->stream, cudaStreamNonBlocking),
                   "cudaStreamCreateWithFlags(eye)"))
    {
        delete impl;
        return false;
    }
    if (!checkCuda(cudaEventCreate(&impl->kernel_start),
                   "cudaEventCreate(eye kernel start)") ||
        !checkCuda(cudaEventCreate(&impl->kernel_end),
                   "cudaEventCreate(eye kernel end)") ||
        !checkCuda(cudaEventCreate(&impl->copy_start),
                   "cudaEventCreate(eye copy start)") ||
        !checkCuda(cudaEventCreate(&impl->copy_end),
                   "cudaEventCreate(eye copy end)"))
    {
        delete impl;
        return false;
    }

    impl->slots.resize(slot_count);
    for (auto &slot : impl->slots)
    {
        if (!checkCuda(cudaMalloc(reinterpret_cast<void **>(&slot.device_left),
                                  impl->rgb_bytes),
                       "cudaMalloc(left RGB)") ||
            !checkCuda(cudaMalloc(reinterpret_cast<void **>(&slot.device_right),
                                  impl->rgb_bytes),
                       "cudaMalloc(right RGB)") ||
            !checkCuda(cudaHostAlloc(&slot.host_left, impl->rgb_bytes,
                                     cudaHostAllocPortable),
                       "cudaHostAlloc(left RGB)") ||
            !checkCuda(cudaHostAlloc(&slot.host_right, impl->rgb_bytes,
                                     cudaHostAllocPortable),
                       "cudaHostAlloc(right RGB)"))
        {
            delete impl;
            return false;
        }
    }
    impl_ = impl;
    return true;
}

bool CudaEyeProcessor::process(int input_fd, uint32_t slot)
{
    auto *impl = static_cast<EyeImpl *>(impl_);
    if (!impl || slot >= impl->slots.size())
        return false;
    impl->submitted.fetch_add(1, std::memory_order_relaxed);
    auto fail = [&]() {
        impl->failures.fetch_add(1, std::memory_order_relaxed);
        return false;
    };
    RegisteredSurface *input = impl->get(input_fd);
    if (!input)
        return fail();

    const auto &buffers = impl->slots[slot];
    const dim3 block(16, 16);
    const dim3 grid((impl->output_width + block.x - 1) / block.x,
                    (impl->output_height + block.y - 1) / block.y, 2);
    if (!checkCuda(cudaEventRecord(impl->kernel_start, impl->stream),
                   "cudaEventRecord(eye kernel start)"))
        return fail();
    eyeRgbKernel<<<grid, block, 0, impl->stream>>>(
        static_cast<const unsigned char *>(input->frame.frame.pPitch[0]),
        input->plane_pitch[0],
        static_cast<const unsigned char *>(input->frame.frame.pPitch[1]),
        input->plane_pitch[1],
        buffers.device_left, buffers.device_right,
        static_cast<int>(impl->src_width), static_cast<int>(impl->src_height),
        static_cast<int>(impl->output_width), static_cast<int>(impl->output_height));
    if (!checkCuda(cudaGetLastError(), "eye RGB kernel") ||
        !checkCuda(cudaEventRecord(impl->kernel_end, impl->stream),
                   "cudaEventRecord(eye kernel end)" ) ||
        !checkCuda(cudaEventRecord(impl->copy_start, impl->stream),
                   "cudaEventRecord(eye copy start)") ||
        !checkCuda(cudaMemcpyAsync(buffers.host_left, buffers.device_left,
                                   impl->rgb_bytes, cudaMemcpyDeviceToHost,
                                   impl->stream),
                   "cudaMemcpyAsync(left RGB)") ||
        !checkCuda(cudaMemcpyAsync(buffers.host_right, buffers.device_right,
                                   impl->rgb_bytes, cudaMemcpyDeviceToHost,
                                   impl->stream),
                   "cudaMemcpyAsync(right RGB)") ||
        !checkCuda(cudaEventRecord(impl->copy_end, impl->stream),
                   "cudaEventRecord(eye copy end)") ||
        !checkCuda(cudaStreamSynchronize(impl->stream),
                   "cudaStreamSynchronize(eye)"))
        return fail();

    float kernel_ms = 0.0f;
    if (checkCuda(cudaEventElapsedTime(&kernel_ms, impl->kernel_start, impl->kernel_end),
                  "cudaEventElapsedTime(eye kernel)"))
    {
        impl->kernel_us_total.fetch_add(static_cast<uint64_t>(kernel_ms * 1000.0f),
                                        std::memory_order_relaxed);
    }
    float copy_ms = 0.0f;
    if (checkCuda(cudaEventElapsedTime(&copy_ms, impl->copy_start, impl->copy_end),
                  "cudaEventElapsedTime(eye copy)"))
    {
        impl->device_to_host_us_total.fetch_add(static_cast<uint64_t>(copy_ms * 1000.0f),
                                                std::memory_order_relaxed);
    }
    impl->completed.fetch_add(1, std::memory_order_relaxed);
    return true;
}

uint32_t CudaEyeProcessor::slotCount() const
{
    const auto *impl = static_cast<const EyeImpl *>(impl_);
    return impl ? static_cast<uint32_t>(impl->slots.size()) : 0;
}

const uint8_t *CudaEyeProcessor::leftData(uint32_t slot) const
{
    const auto *impl = static_cast<const EyeImpl *>(impl_);
    return impl && slot < impl->slots.size() ? impl->slots[slot].host_left : nullptr;
}

const uint8_t *CudaEyeProcessor::rightData(uint32_t slot) const
{
    const auto *impl = static_cast<const EyeImpl *>(impl_);
    return impl && slot < impl->slots.size() ? impl->slots[slot].host_right : nullptr;
}

size_t CudaEyeProcessor::rgbBytes() const
{
    const auto *impl = static_cast<const EyeImpl *>(impl_);
    return impl ? impl->rgb_bytes : 0;
}

bool CudaEyeProcessor::isInitialized() const
{
    return impl_ != nullptr;
}

CudaPipelineStats CudaEyeProcessor::consumeStats()
{
    CudaPipelineStats stats;
    auto *impl = static_cast<EyeImpl *>(impl_);
    if (!impl)
        return stats;
    stats.submitted = impl->submitted.exchange(0, std::memory_order_relaxed);
    stats.completed = impl->completed.exchange(0, std::memory_order_relaxed);
    stats.failures = impl->failures.exchange(0, std::memory_order_relaxed);
    stats.kernel_us_total = impl->kernel_us_total.exchange(0, std::memory_order_relaxed);
    stats.device_to_host_us_total = impl->device_to_host_us_total.exchange(0, std::memory_order_relaxed);
    return stats;
}

} // namespace trb::video
