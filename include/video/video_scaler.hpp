#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>

namespace trb::video
{

// Synchronously resize a pitch-linear NV12 DMA-BUF into a pool-owned
// pitch-linear NV12 DMA-BUF. The transform is intentionally dimension-only:
// callers must validate the aspect ratio before initializing this component.
class VideoScaler
{
public:
    struct Config
    {
        uint32_t input_width = 3840;
        uint32_t input_height = 1520;
        uint32_t output_width = 3840;
        uint32_t output_height = 1520;
        uint32_t buffer_pool_size = 4;
    };

    VideoScaler();
    ~VideoScaler();

    VideoScaler(const VideoScaler &) = delete;
    VideoScaler &operator=(const VideoScaler &) = delete;

    bool initialize(const Config &config);

    // On success, output_fd_out belongs to this scaler until releaseFd() is
    // called. The source DMA-BUF remains owned by the caller.
    bool transformSync(int input_fd, int &output_fd_out);

    void releaseFd(int dmabuf_fd);

private:
    void destroyBuffers();

    Config config_;
    std::vector<void *> surfaces_;
    std::vector<int> dmabuf_fds_;
    std::mutex pool_mutex_;
    std::queue<size_t> free_indices_;
    std::unordered_map<int, size_t> fd_to_index_;
};

} // namespace trb::video
