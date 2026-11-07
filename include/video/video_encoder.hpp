#pragma once

#include <functional>
#include <memory>
#include <atomic>
#include <queue>
#include <mutex>
#include <vector>
#include <map>
#include <cstdint>
#include <cstddef>

#include "NvVideoEncoder.h"

namespace trb::video
{

    /**
     * Synchronous wrapper around NvVideoEncoder.
     *
     * Threading model: caller drives the encoder from a single thread via:
     *   submit(fd, ts)       \u2192 push an NV12 DMA-BUF onto the output plane.
     *   dequeueOne(pkt, ms)  \u2192 pull one encoded AU off the capture plane.
     *   releaseCapture(pkt)  \u2192 requeue the capture buffer after consumption.
     *   drainInputDone()     \u2192 non-blocking reap of finished output-plane
     *                          buffers; invokes input_done_callback_ for each.
     */
    class VideoEncoder
    {
    public:
        // Future-proofing for H.265. Only kH264 is implemented today; constructing
        // with kH265 logs and fails initialize().
        enum class Codec
        {
            kH264 = 0,
            kH265 = 1,
        };

        struct Config
        {
            Codec codec = Codec::kH264;

            uint32_t width;
            uint32_t height;
            uint32_t framerate;
            uint32_t bitrate; // bits per second
            uint32_t idr_interval = 30;
            uint32_t idr_interval_gops = 30;
            uint32_t slice_intra_refresh_interval_slices = 0;
            uint32_t force_idr_every_n = 0;
            uint32_t qp_range_i_min = 10;
            uint32_t qp_range_i_max = 40;
            uint32_t qp_range_p_min = 10;
            uint32_t qp_range_p_max = 40;

            bool max_perf_mode = false;
            int32_t hw_preset_type = -1;
            int32_t rate_control_mode = -1;
            uint32_t peak_bitrate = 0;
            uint32_t virtual_buffer_size = 0;
            int32_t num_reference_frames = -1;
            int32_t num_b_frames = -1;
            int32_t insert_sps_pps_at_idr = -1;
            bool slice_level_encode = true;
            // Multi-slice per frame for low-latency emission. With
            // slice_level_encode=true, NVENC emits each slice as a separate
            // AU as soon as it is encoded, so encode latency drops roughly
            // proportional to num_slices. 0 / 1 = single slice per frame.
            uint32_t num_slices = 0;
            int32_t poc_type = -1;
            bool insert_aud = false;

            uint32_t output_plane_buffers = 10;
            uint32_t capture_plane_buffers = 10;

            uint32_t encoder_framerate_override = 0;
        };

        // One dequeued encoded access unit. Must be returned via releaseCapture().
        // |data|/|size| reference memory owned by the encoder; valid only until
        // releaseCapture() is called.
        struct EncodedPacket
        {
            const uint8_t* data{nullptr};
            size_t         size{0};
            uint64_t       timestamp_us{0};
            bool           keyframe{false};  // true for IDR
            // Internal bookkeeping \u2013 do not modify.
            uint32_t  _cap_buf_index{0};
            NvBuffer* _nvbuf{nullptr};
        };

        using InputDoneCallback = std::function<void(int dmabuf_fd)>;
        // Called once when the codec parameter sets have been captured. NALs do
        // NOT include start codes. For H.264 |vps| is empty; for H.265 |vps|,
        // |sps| and |pps| are all populated.
        using SpsPpsCallback = std::function<void(const std::vector<uint8_t>& sps,
                                                  const std::vector<uint8_t>& pps,
                                                  const std::vector<uint8_t>& vps)>;

        VideoEncoder();
        ~VideoEncoder();

        VideoEncoder(const VideoEncoder&) = delete;
        VideoEncoder& operator=(const VideoEncoder&) = delete;

        bool initialize(const Config& config);

        void setInputDoneCallback(InputDoneCallback callback);
        void setSpsPpsCallback(SpsPpsCallback callback);

        // Feed one NV12 DMA-BUF to the encoder. Returns false when the output
        // plane queue is full (caller should drainInputDone() and retry later).
        bool submit(int dmabuf_fd, uint64_t timestamp_us);

        // Try to pull one encoded AU. timeout_ms=0 means non-blocking.
        // Returns true and fills |out| on success; caller MUST call
        // releaseCapture(out) before the next dequeueOne()/destruction.
        bool dequeueOne(EncodedPacket& out, int timeout_ms);

        // Return the capture buffer referenced by |pkt| to the encoder hardware.
        void releaseCapture(EncodedPacket& pkt);

        // Non-blocking: reap any finished output-plane buffers, invoking
        // input_done_callback_ for each released input DMA-BUF.
        void drainInputDone();

        // Update encoder bitrate at runtime (bps). Returns false on failure.
        bool setBitrate(uint32_t bitrate_bps);

    private:
        void parseSpsPps_(const uint8_t* data, size_t size);

        Config config_;
        InputDoneCallback input_done_callback_;
        SpsPpsCallback sps_pps_callback_;
        std::atomic<bool> sps_pps_sent_{false};
        std::vector<uint8_t> cached_sps_;
        std::vector<uint8_t> cached_pps_;
        std::vector<uint8_t> cached_vps_;  // H.265 only
        std::mutex sps_pps_mutex_;
        std::unique_ptr<NvVideoEncoder> encoder_;

        std::mutex input_mutex_;
        std::queue<int> free_output_indices_;

        std::mutex bitrate_mutex_;

        std::mutex map_mutex_;
        std::map<int, int> index_to_fd_;

        // Timestamps queued by submit(); popped per-AU by dequeueOne(). With
        // multi-slice encoding NVENC zeroes the capture-plane v4l2 timestamp,
        // so we must replay submit() timestamps in-order. submit() pushes
        // one entry per frame; the first AU of the frame pops it and stores
        // it in |last_capture_ts_|, subsequent slice AUs reuse that value.
        std::mutex ts_queue_mutex_;
        std::queue<uint64_t> timestamp_queue_;
        uint64_t last_capture_ts_ = 0;
        uint32_t effective_slices_per_frame_ = 1;

        std::atomic<uint64_t> input_frame_count_{0};
    };

} // namespace trb::video
