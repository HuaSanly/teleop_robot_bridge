/**
 * @file nvbuf_mutex.hpp
 * @brief Global mutex for serializing NvBufSurface/VIC hardware operations
 * 
 * The VIC (Video Image Compositor) hardware on Jetson is NOT thread-safe.
 * All NvBufSurfTransform operations must be serialized across threads.
 */

#pragma once

#include <mutex>

namespace trb::video
{

/**
 * @brief Global mutex for VIC/NvBufSurface operations
 * 
 * Use this lock around any of the following operations:
 * - NvBufSurfTransform()
 * - NvBufSurfaceMap() / NvBufSurfaceUnMap()
 * - NvBufSurfaceSyncForCpu() / NvBufSurfaceSyncForDevice()
 * - NvBufSurfaceFromFd()
 */
inline std::mutex& getNvBufMutex()
{
    static std::mutex mutex;
    return mutex;
}

} // namespace trb::video
