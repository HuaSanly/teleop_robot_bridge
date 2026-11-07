#include "video/video_encoder.hpp"
#include "video/nvbuf_mutex.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <atomic>
#include <fcntl.h>
#include <cerrno>

#include <linux/v4l2-controls.h>

#include "NvUtils.h"
#include "nvbufsurface.h"
#include "v4l2_nv_extensions.h"

namespace trb::video
{

    namespace
    {
        enum class H264StreamFormat
        {
            AnnexB,
            Avcc,
            Unknown,
        };

        class BitReader
        {
        public:
            BitReader(const uint8_t *data, size_t size) : data_(data), size_(size) {}

            bool readBit(uint32_t &out)
            {
                if (bitpos_ >= size_ * 8)
                {
                    return false;
                }
                const size_t byte_i = bitpos_ / 8;
                const uint32_t bit_i = 7u - static_cast<uint32_t>(bitpos_ % 8);
                out = (data_[byte_i] >> bit_i) & 1u;
                ++bitpos_;
                return true;
            }

            bool readBits(uint32_t n, uint32_t &out)
            {
                out = 0;
                for (uint32_t i = 0; i < n; ++i)
                {
                    uint32_t b = 0;
                    if (!readBit(b))
                    {
                        return false;
                    }
                    out = (out << 1) | b;
                }
                return true;
            }

            bool readUE(uint32_t &out)
            {
                // Exp-Golomb (unsigned)
                uint32_t zeros = 0;
                while (true)
                {
                    uint32_t b = 0;
                    if (!readBit(b))
                    {
                        return false;
                    }
                    if (b == 0)
                    {
                        ++zeros;
                        if (zeros > 31)
                        {
                            return false;
                        }
                        continue;
                    }
                    break;
                }

                uint32_t suffix = 0;
                if (zeros > 0)
                {
                    if (!readBits(zeros, suffix))
                    {
                        return false;
                    }
                }
                out = ((1u << zeros) - 1u) + suffix;
                return true;
            }

        private:
            const uint8_t *data_{nullptr};
            size_t size_{0};
            size_t bitpos_{0};
        };

        static size_t rbspPrefix(const uint8_t *src, size_t src_size, uint8_t *dst, size_t dst_cap)
        {
            // Copy while removing emulation-prevention bytes (00 00 03).
            size_t w = 0;
            uint32_t zero_count = 0;
            for (size_t i = 0; i < src_size && w < dst_cap; ++i)
            {
                const uint8_t b = src[i];
                if (zero_count >= 2 && b == 0x03)
                {
                    zero_count = 0;
                    continue;
                }
                dst[w++] = b;
                if (b == 0x00)
                {
                    ++zero_count;
                }
                else
                {
                    zero_count = 0;
                }
            }
            return w;
        }

        static bool hasAnnexBStartCode(const uint8_t *data, size_t size)
        {
            if (!data || size < 4)
            {
                return false;
            }
            for (size_t i = 0; i + 3 < size; ++i)
            {
                if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01)
                {
                    return true;
                }
                if (i + 4 < size && data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x00 && data[i + 3] == 0x01)
                {
                    return true;
                }
            }
            return false;
        }

        static bool looksLikeAvcc(const uint8_t *data, size_t size)
        {
            // Heuristic: try to parse a few 4-byte length-prefixed NALs without overrun.
            if (!data || size < 8)
            {
                return false;
            }
            size_t off = 0;
            int nal_count = 0;
            while (off + 4 <= size && nal_count < 3)
            {
                const uint32_t n = (static_cast<uint32_t>(data[off]) << 24) |
                                   (static_cast<uint32_t>(data[off + 1]) << 16) |
                                   (static_cast<uint32_t>(data[off + 2]) << 8) |
                                   (static_cast<uint32_t>(data[off + 3]));
                off += 4;
                if (n == 0 || off + n > size)
                {
                    return false;
                }
                // NAL header must exist.
                if (n < 1)
                {
                    return false;
                }
                off += n;
                ++nal_count;
            }
            return nal_count > 0;
        }

        static H264StreamFormat detectH264Format(const uint8_t *data, size_t size)
        {
            if (hasAnnexBStartCode(data, size))
            {
                return H264StreamFormat::AnnexB;
            }
            if (looksLikeAvcc(data, size))
            {
                return H264StreamFormat::Avcc;
            }
            return H264StreamFormat::Unknown;
        }

        template <typename Fn>
        static void forEachNal(const uint8_t *data, size_t size, H264StreamFormat fmt, Fn &&fn)
        {
            if (!data || size < 1)
            {
                return;
            }

            if (fmt == H264StreamFormat::AnnexB)
            {
                auto is_start_code = [&](size_t i, size_t &sc_len) -> bool
                {
                    if (i + 3 <= size && data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01)
                    {
                        sc_len = 3;
                        return true;
                    }
                    if (i + 4 <= size && data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x00 && data[i + 3] == 0x01)
                    {
                        sc_len = 4;
                        return true;
                    }
                    return false;
                };

                size_t i = 0;
                while (i + 3 < size)
                {
                    size_t sc_len = 0;
                    if (!is_start_code(i, sc_len))
                    {
                        ++i;
                        continue;
                    }
                    const size_t nal_hdr = i + sc_len;
                    if (nal_hdr >= size)
                    {
                        i = nal_hdr;
                        continue;
                    }
                    size_t nal_end = size;
                    for (size_t j = nal_hdr + 1; j + 3 < size; ++j)
                    {
                        size_t next_sc = 0;
                        if (is_start_code(j, next_sc))
                        {
                            nal_end = j;
                            break;
                        }
                    }
                    fn(data + nal_hdr, nal_end - nal_hdr);
                    i = nal_hdr;
                }
                return;
            }

            if (fmt == H264StreamFormat::Avcc)
            {
                size_t off = 0;
                while (off + 4 <= size)
                {
                    const uint32_t n = (static_cast<uint32_t>(data[off]) << 24) |
                                       (static_cast<uint32_t>(data[off + 1]) << 16) |
                                       (static_cast<uint32_t>(data[off + 2]) << 8) |
                                       (static_cast<uint32_t>(data[off + 3]));
                    off += 4;
                    if (n == 0 || off + n > size)
                    {
                        return;
                    }
                    fn(data + off, static_cast<size_t>(n));
                    off += n;
                }
            }
        }

        // Detect IDR NAL units in an H.264 bytestream (Annex-B or AVCC).
        // Returns true if any NAL unit with nal_unit_type==5 is found.
        static bool containsH264IdrNal(const uint8_t *data, size_t size)
        {
            const H264StreamFormat fmt = detectH264Format(data, size);
            bool found = false;
            forEachNal(data, size, fmt, [&](const uint8_t *nal, size_t nal_size)
                       {
                           if (nal_size < 1)
                               return;
                           const uint8_t nal_unit_type = static_cast<uint8_t>(nal[0] & 0x1Fu);
                           if (nal_unit_type == 5)
                           {
                               found = true;
                           } });
            return found;
        }

        // Detect IDR NAL units in an H.265 (HEVC) bytestream. The HEVC NAL
        // header is 2 bytes; nal_unit_type lives in bits 1..6 of byte 0.
        // IDR_W_RADL=19, IDR_N_LP=20 (both are IDR random-access points).
        // CRA_NUT=21 is also a clean random-access point and treated as a
        // keyframe for transport purposes.
        static bool containsH265IdrNal(const uint8_t *data, size_t size)
        {
            const H264StreamFormat fmt = detectH264Format(data, size);
            bool found = false;
            forEachNal(data, size, fmt, [&](const uint8_t *nal, size_t nal_size)
                       {
                           if (nal_size < 2)
                               return;
                           const uint8_t nal_unit_type = static_cast<uint8_t>((nal[0] >> 1) & 0x3Fu);
                           if (nal_unit_type == 19 || nal_unit_type == 20 || nal_unit_type == 21)
                           {
                               found = true;
                           } });
            return found;
        }

        // Detect whether the bitstream contains any I-slice (IDR or non-IDR).
        // For nal_unit_type 1 or 5, parse slice_type and check if it's an I slice.
        static bool containsH264ISlice(const uint8_t *data, size_t size)
        {
            const H264StreamFormat fmt = detectH264Format(data, size);
            bool found = false;
            forEachNal(data, size, fmt, [&](const uint8_t *nal, size_t nal_size)
                       {
                           if (found)
                               return;
                           if (nal_size < 2)
                               return;

                           const uint8_t nal_unit_type = static_cast<uint8_t>(nal[0] & 0x1Fu);
                           if (nal_unit_type != 1 && nal_unit_type != 5)
                               return;

                           const uint8_t *payload = nal + 1;
                           const size_t payload_size = nal_size - 1;

                           uint8_t rbsp[96];
                           const size_t rbsp_n = rbspPrefix(payload, payload_size, rbsp, sizeof(rbsp));
                           BitReader br(rbsp, rbsp_n);
                           uint32_t first_mb = 0;
                           uint32_t slice_type = 0;
                           if (!br.readUE(first_mb) || !br.readUE(slice_type))
                               return;

                           // slice_type mod 5: 0=P,1=B,2=I,3=SP,4=SI
                           if ((slice_type % 5u) == 2u)
                           {
                               found = true;
                           } });
            return found;
        }

        static const char *h264FormatToString(H264StreamFormat fmt)
        {
            switch (fmt)
            {
            case H264StreamFormat::AnnexB:
                return "annexb";
            case H264StreamFormat::Avcc:
                return "avcc";
            default:
                return "unknown";
            }
        }

        static std::string bytesToHex(const uint8_t *data, size_t size)
        {
            static const char kHex[] = "0123456789abcdef";
            std::string out;
            out.reserve(size * 2);
            for (size_t i = 0; i < size; ++i)
            {
                const uint8_t b = data[i];
                out.push_back(kHex[(b >> 4) & 0x0F]);
                out.push_back(kHex[b & 0x0F]);
            }
            return out;
        }

        static std::string base64Encode(const uint8_t *data, size_t size)
        {
            static const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            std::string out;
            out.reserve(((size + 2) / 3) * 4);

            size_t i = 0;
            while (i + 3 <= size)
            {
                const uint32_t v = (static_cast<uint32_t>(data[i]) << 16) |
                                   (static_cast<uint32_t>(data[i + 1]) << 8) |
                                   (static_cast<uint32_t>(data[i + 2]));
                out.push_back(kB64[(v >> 18) & 0x3F]);
                out.push_back(kB64[(v >> 12) & 0x3F]);
                out.push_back(kB64[(v >> 6) & 0x3F]);
                out.push_back(kB64[v & 0x3F]);
                i += 3;
            }

            const size_t rem = size - i;
            if (rem == 1)
            {
                const uint32_t v = (static_cast<uint32_t>(data[i]) << 16);
                out.push_back(kB64[(v >> 18) & 0x3F]);
                out.push_back(kB64[(v >> 12) & 0x3F]);
                out.push_back('=');
                out.push_back('=');
            }
            else if (rem == 2)
            {
                const uint32_t v = (static_cast<uint32_t>(data[i]) << 16) |
                                   (static_cast<uint32_t>(data[i + 1]) << 8);
                out.push_back(kB64[(v >> 18) & 0x3F]);
                out.push_back(kB64[(v >> 12) & 0x3F]);
                out.push_back(kB64[(v >> 6) & 0x3F]);
                out.push_back('=');
            }
            return out;
        }

        static void maybePrintSpsPpsOnce(const uint8_t *data, size_t size)
        {
            static std::atomic<bool> printed{false};
            static std::mutex mu;
            static std::vector<uint8_t> sps;
            static std::vector<uint8_t> pps;

            if (printed.load(std::memory_order_relaxed))
            {
                return;
            }

            const H264StreamFormat fmt = detectH264Format(data, size);
            forEachNal(data, size, fmt, [&](const uint8_t *nal, size_t nal_size)
                       {
                           if (nal_size < 1)
                               return;
                           const uint8_t nal_unit_type = static_cast<uint8_t>(nal[0] & 0x1Fu);
                           if (nal_unit_type != 7 && nal_unit_type != 8)
                               return;

                           std::lock_guard<std::mutex> lk(mu);
                           if (printed.load(std::memory_order_relaxed))
                               return;

                           if (nal_unit_type == 7 && sps.empty())
                           {
                               sps.assign(nal, nal + nal_size);
                           }
                           else if (nal_unit_type == 8 && pps.empty())
                           {
                               pps.assign(nal, nal + nal_size);
                           }

                           if (!sps.empty() && !pps.empty())
                           {
                               printed.store(true, std::memory_order_relaxed);
                               std::cerr
                                   << "Encoder SPS/PPS captured once: format=" << h264FormatToString(fmt)
                                   << " sps_bytes=" << sps.size()
                                   << " pps_bytes=" << pps.size()
                                   << std::endl;
                           } });
        }
    }

    VideoEncoder::VideoEncoder() = default;

    VideoEncoder::~VideoEncoder()
    {
        encoder_.reset();
    }

    bool VideoEncoder::initialize(const Config &config)
    {
        config_ = config;
        const bool is_h265 = (config_.codec == Codec::kH265);

        // O_NONBLOCK makes NvV4l2Element::dqBuffer's num_retries argument act as
        // a millisecond timeout instead of a blocking retry counter.
        encoder_.reset(NvVideoEncoder::createVideoEncoder("enc0", O_NONBLOCK));
        if (!encoder_)
        {
            std::cerr << "Failed to create NvVideoEncoder" << std::endl;
            return false;
        }

        const uint32_t capture_pixfmt = is_h265 ? V4L2_PIX_FMT_H265 : V4L2_PIX_FMT_H264;
        if (encoder_->setCapturePlaneFormat(capture_pixfmt, config_.width, config_.height, 4 * 1024 * 1024) < 0)
        {
            std::cerr << "Failed to set capture plane format" << std::endl;
            return false;
        }

        // Set output plane format (Raw input)
        if (encoder_->setOutputPlaneFormat(V4L2_PIX_FMT_NV12M, config_.width, config_.height) < 0)
        {
            std::cerr << "Failed to set output plane format" << std::endl;
            return false;
        }

        if (encoder_->setBitrate(config_.bitrate) < 0)
        {
            std::cerr << "Failed to set bitrate" << std::endl;
            return false;
        }

        // -------- Low-latency/perf critical controls (must be before requestBuffers) --------
        // Notes from Jetson MM API docs:
        // - Must be called after setFormat on both planes and before requestBuffers/setupPlane.
        if (config_.max_perf_mode)
        {
            if (encoder_->setMaxPerfMode(1) < 0)
            {
                std::cerr << "WARN: Failed to enable max perf mode" << std::endl;
            }
        }
        if (config_.hw_preset_type >= 0)
        {
            if (encoder_->setHWPresetType(static_cast<v4l2_enc_hw_preset_type>(config_.hw_preset_type)) < 0)
            {
                std::cerr << "WARN: Failed to set HW preset type" << std::endl;
            }
        }

        if (config_.rate_control_mode >= 0)
        {
            if (encoder_->setRateControlMode(static_cast<v4l2_mpeg_video_bitrate_mode>(config_.rate_control_mode)) < 0)
            {
                std::cerr << "WARN: Failed to set rate control mode" << std::endl;
            }
        }

        if (config_.peak_bitrate > 0)
        {
            if (encoder_->setPeakBitrate(config_.peak_bitrate) < 0)
            {
                std::cerr << "WARN: Failed to set peak bitrate" << std::endl;
            }
        }

        if (config_.virtual_buffer_size > 0)
        {
            if (encoder_->setVirtualBufferSize(config_.virtual_buffer_size) < 0)
            {
                std::cerr << "WARN: Failed to set virtual buffer size" << std::endl;
            }
        }

        if (config_.slice_intra_refresh_interval_slices > 0)
        {
            if (encoder_->setSliceIntrarefresh(config_.slice_intra_refresh_interval_slices) < 0)
            {
                std::cerr << "WARN: Failed to set slice intra-refresh interval" << std::endl;
            }
        }

        if (config_.num_reference_frames >= 0)
        {
            if (encoder_->setNumReferenceFrames(static_cast<uint32_t>(config_.num_reference_frames)) < 0)
            {
                std::cerr << "WARN: Failed to set num reference frames" << std::endl;
            }
        }

        if (config_.num_b_frames >= 0)
        {
            if (encoder_->setNumBFrames(static_cast<uint32_t>(config_.num_b_frames)) < 0)
            {
                std::cerr << "WARN: Failed to set num B-frames" << std::endl;
            }
        }

        if (config_.insert_sps_pps_at_idr >= 0)
        {
            const bool enabled = (config_.insert_sps_pps_at_idr != 0);
            if (encoder_->setInsertSpsPpsAtIdrEnabled(enabled) < 0)
            {
                std::cerr << "WARN: Failed to set insert SPS/PPS at IDR" << std::endl;
            }
        }

        // Low-latency: emit each slice as it finishes instead of waiting for
        // the next frame to flush the encoder pipeline. Without this the
        // measured encode latency is pinned to ~1 frame period (~33 ms @ 30fps).
        if (encoder_->setSliceLevelEncode(config_.slice_level_encode) < 0)
        {
            std::cerr << "WARN: Failed to set slice level encode" << std::endl;
        }

        // Multi-slice configuration: split each frame into N macroblock-equal
        // slices. Combined with slice_level_encode, this lets the first slice
        // be DQ'd while the rest of the frame is still encoding, cutting the
        // submit→AU latency floor from ~frame_period/1 to ~frame_period/N.
        if (config_.slice_level_encode && config_.num_slices > 1)
        {
            const uint32_t mbs_w = (config_.width  + 15u) / 16u;
            const uint32_t mbs_h = (config_.height + 15u) / 16u;
            const uint32_t total_mbs = mbs_w * mbs_h;
            const uint32_t n = config_.num_slices;
            const uint32_t per_slice_mbs = std::max(1u, (total_mbs + n - 1u) / n);
            if (encoder_->setSliceLength(V4L2_ENC_SLICE_LENGTH_TYPE_MBLK,
                                         per_slice_mbs) < 0)
            {
                std::cerr << "WARN: Failed to set slice length (MBLK="
                          << per_slice_mbs << ", num_slices=" << n << ")"
                          << std::endl;
                effective_slices_per_frame_ = 1;
            }
            else
            {
                effective_slices_per_frame_ = n;
            }
        }
        else
        {
            effective_slices_per_frame_ = 1;
        }

        if (config_.poc_type >= 0)
        {
            // POC type is an H.264-only concept (poc_type 0/1/2). HEVC has no
            // equivalent control; silently skip when encoding HEVC.
            if (!is_h265)
            {
                if (encoder_->setPocType(static_cast<uint32_t>(config_.poc_type)) < 0)
                {
                    std::cerr << "WARN: Failed to set POC type" << std::endl;
                }
            }
        }

        if (config_.insert_aud)
        {
            if (encoder_->setInsertAudEnabled(true) < 0)
            {
                std::cerr << "WARN: Failed to enable AUD insertion" << std::endl;
            }
        }

        const uint32_t profile = is_h265
                                     ? static_cast<uint32_t>(V4L2_MPEG_VIDEO_H265_PROFILE_MAIN)
                                     : static_cast<uint32_t>(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
        if (encoder_->setProfile(profile) < 0)
        {
            std::cerr << "Failed to set profile" << std::endl;
            return false;
        }

        const uint32_t level = is_h265
                                   ? static_cast<uint32_t>(V4L2_MPEG_VIDEO_H265_LEVEL_5_1_MAIN_TIER)
                                   : static_cast<uint32_t>(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
        if (encoder_->setLevel(level) < 0)
        {
            std::cerr << "Failed to set level" << std::endl;
            return false;
        }

        // Set QP range for quality control (lower QP = higher quality)
        if (config_.qp_range_i_min > 0 || config_.qp_range_i_max > 0 ||
            config_.qp_range_p_min > 0 || config_.qp_range_p_max > 0)
        {
            // B-frame uses P-frame QP range since we have num_b_frames=0
            if (encoder_->setQpRange(config_.qp_range_i_min, config_.qp_range_i_max,
                                     config_.qp_range_p_min, config_.qp_range_p_max,
                                     config_.qp_range_p_min, config_.qp_range_p_max) < 0)
            {
                std::cerr << "WARN: Failed to set QP range (I: " << config_.qp_range_i_min << "-" << config_.qp_range_i_max
                          << ", P: " << config_.qp_range_p_min << "-" << config_.qp_range_p_max << ")" << std::endl;
            }
        }

        const uint32_t enc_fps_num = (config_.encoder_framerate_override > 0)
                                         ? config_.encoder_framerate_override
                                         : config_.framerate;
        if (encoder_->setFrameRate(enc_fps_num, 1) < 0)
        {
            std::cerr << "Failed to set framerate" << std::endl;
            return false;
        }

        if (encoder_->setIDRInterval(config_.idr_interval_gops) < 0)
        {
            std::cerr << "Failed to set IDR interval" << std::endl;
            return false;
        }

        if (encoder_->setIFrameInterval(config_.idr_interval) < 0)
        {
            std::cerr << "Failed to set IFrame interval" << std::endl;
            return false;
        }

        const uint32_t output_plane_buffers = (config_.output_plane_buffers > 0) ? config_.output_plane_buffers : 10;
        const uint32_t capture_plane_buffers = (config_.capture_plane_buffers > 0) ? config_.capture_plane_buffers : 10;

        // Setup planes
        // Output plane: DMABUF (we feed FDs)
        if (encoder_->output_plane.setupPlane(V4L2_MEMORY_DMABUF, output_plane_buffers, true, false) < 0)
        {
            std::cerr << "Failed to setup output plane" << std::endl;
            return false;
        }

        // Capture plane: MMAP (we read encoded data from mapped buffers)
        if (encoder_->capture_plane.setupPlane(V4L2_MEMORY_MMAP, capture_plane_buffers, true, false) < 0)
        {
            std::cerr << "Failed to setup capture plane" << std::endl;
            return false;
        }

        // Stream On
        if (encoder_->output_plane.setStreamStatus(true) < 0)
        {
            std::cerr << "Failed to stream on output plane" << std::endl;
            return false;
        }

        if (encoder_->capture_plane.setStreamStatus(true) < 0)
        {
            std::cerr << "Failed to stream on capture plane" << std::endl;
            return false;
        }

        // Queue all capture plane buffers
        for (uint32_t i = 0; i < encoder_->capture_plane.getNumBuffers(); ++i)
        {
            struct v4l2_buffer v4l2_buf;
            struct v4l2_plane planes[MAX_PLANES];
            memset(&v4l2_buf, 0, sizeof(v4l2_buf));
            memset(planes, 0, sizeof(planes));

            v4l2_buf.index = i;
            v4l2_buf.m.planes = planes;
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            v4l2_buf.memory = V4L2_MEMORY_MMAP;
            v4l2_buf.length = encoder_->capture_plane.getNumPlanes();

            if (encoder_->capture_plane.qBuffer(v4l2_buf, nullptr) < 0)
            {
                std::cerr << "Failed to qBuffer on capture plane" << std::endl;
                return false;
            }
        }

        // Initialize free output indices
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            while (!free_output_indices_.empty())
                free_output_indices_.pop();
            for (uint32_t i = 0; i < encoder_->output_plane.getNumBuffers(); ++i)
            {
                free_output_indices_.push(i);
            }
        }

        return true;
    }

    void VideoEncoder::setInputDoneCallback(InputDoneCallback callback)
    {
        input_done_callback_ = std::move(callback);
    }

    void VideoEncoder::setSpsPpsCallback(SpsPpsCallback callback)
    {
        std::lock_guard<std::mutex> lk(sps_pps_mutex_);
        sps_pps_callback_ = std::move(callback);
        const bool is_h265 = (config_.codec == Codec::kH265);
        const bool ready = !cached_sps_.empty() && !cached_pps_.empty() &&
                           (!is_h265 || !cached_vps_.empty());
        if (ready && sps_pps_callback_)
        {
            sps_pps_callback_(cached_sps_, cached_pps_, cached_vps_);
            sps_pps_sent_.store(true);
        }
    }

    bool VideoEncoder::submit(int dmabuf_fd, uint64_t timestamp_us)
    {
        if (!encoder_)
            return false;

        // NOTE: drainInputDone() is intentionally NOT called here. The
        // VideoStreamManager encode loop already calls drainInputDone() once
        // per iteration before submit; calling it again here doubled the
        // dq lock-thrash with no benefit.

        int index = -1;
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            if (free_output_indices_.empty())
            {
                return false;
            }
            index = free_output_indices_.front();
            free_output_indices_.pop();
        }

        NvBuffer *buffer = encoder_->output_plane.getNthBuffer(index);
        if (!buffer)
        {
            std::cerr << "VideoEncoder: Failed to get buffer " << index << std::endl;
            std::lock_guard<std::mutex> lock(input_mutex_);
            free_output_indices_.push(index);
            return false;
        }

        NvBufSurface *surf = nullptr;
        {
            std::lock_guard<std::mutex> lk(trb::video::getNvBufMutex());
            if (NvBufSurfaceFromFd(dmabuf_fd, (void **)&surf) != 0 || surf == nullptr)
            {
                std::cerr << "VideoEncoder: NvBufSurfaceFromFd failed" << std::endl;
                std::lock_guard<std::mutex> lock(input_mutex_);
                free_output_indices_.push(index);
                return false;
            }
        }

        if (encoder_->output_plane.getNumPlanes() != surf->surfaceList[0].planeParams.num_planes)
        {
            std::cerr << "VideoEncoder: plane count mismatch: enc_planes="
                      << encoder_->output_plane.getNumPlanes() << " surf_planes="
                      << surf->surfaceList[0].planeParams.num_planes << std::endl;
            std::lock_guard<std::mutex> lock(input_mutex_);
            free_output_indices_.push(index);
            return false;
        }

        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.index = index;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        v4l2_buf.memory = V4L2_MEMORY_DMABUF;
        v4l2_buf.length = encoder_->output_plane.getNumPlanes();
        v4l2_buf.timestamp.tv_sec = timestamp_us / 1000000;
        v4l2_buf.timestamp.tv_usec = timestamp_us % 1000000;

        const uint64_t in_count = input_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (config_.force_idr_every_n > 0 && (in_count % config_.force_idr_every_n) == 0)
        {
            (void)encoder_->forceIDR();
        }

        const auto &planeParams = surf->surfaceList[0].planeParams;
        for (uint32_t i = 0; i < v4l2_buf.length; ++i)
        {
            planes[i].m.fd = dmabuf_fd;
            if (i < planeParams.num_planes)
            {
                planes[i].bytesused = planeParams.psize[i];
                planes[i].data_offset = planeParams.offset[i];
            }
            else
            {
                planes[i].bytesused = 0;
                planes[i].data_offset = 0;
            }
        }

        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            index_to_fd_[index] = dmabuf_fd;
        }
        {
            // Push exactly one timestamp per submit. The first AU of this
            // frame pops it; subsequent slice AUs reuse |last_capture_ts_|
            // (set in dequeueOne after the first AU pops). NVENC zeroes the
            // capture-plane v4l2 timestamp on non-first slices, so we cannot
            // rely on it.
            std::lock_guard<std::mutex> lk(ts_queue_mutex_);
            timestamp_queue_.push(timestamp_us);
        }

        if (encoder_->output_plane.qBuffer(v4l2_buf, nullptr) < 0)
        {
            std::cerr << "VideoEncoder: Failed to qBuffer" << std::endl;
            {
                std::lock_guard<std::mutex> lk(map_mutex_);
                index_to_fd_.erase(index);
            }
            {
                std::lock_guard<std::mutex> lk(ts_queue_mutex_);
                if (!timestamp_queue_.empty())
                    timestamp_queue_.pop();
            }
            std::lock_guard<std::mutex> lock(input_mutex_);
            free_output_indices_.push(index);
            return false;
        }

        return true;
    }

    void VideoEncoder::drainInputDone()
    {
        if (!encoder_)
            return;

        while (true)
        {
            struct v4l2_buffer v4l2_buf;
            struct v4l2_plane  planes[MAX_PLANES];
            memset(&v4l2_buf, 0, sizeof(v4l2_buf));
            memset(planes, 0, sizeof(planes));
            v4l2_buf.m.planes = planes;

            NvBuffer *buf = nullptr;
            // num_retries=0 in O_NONBLOCK mode => non-blocking single attempt.
            if (encoder_->output_plane.dqBuffer(v4l2_buf, &buf, nullptr, 0) < 0)
            {
                break;  // EAGAIN or error: nothing pending.
            }

            int fd = -1;
            {
                std::lock_guard<std::mutex> lk(map_mutex_);
                auto it = index_to_fd_.find(v4l2_buf.index);
                if (it != index_to_fd_.end())
                {
                    fd = it->second;
                    index_to_fd_.erase(it);
                }
            }

            if (fd >= 0 && input_done_callback_)
            {
                input_done_callback_(fd);
            }

            {
                std::lock_guard<std::mutex> lock(input_mutex_);
                free_output_indices_.push(v4l2_buf.index);
            }
        }
    }

    bool VideoEncoder::dequeueOne(EncodedPacket &out, int timeout_ms)
    {
        if (!encoder_)
            return false;

        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane  planes[MAX_PLANES];
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));
        v4l2_buf.m.planes = planes;

        const uint32_t retries = static_cast<uint32_t>(std::max(0, timeout_ms));
        NvBuffer *buf = nullptr;
        if (encoder_->capture_plane.dqBuffer(v4l2_buf, &buf, nullptr, retries) < 0)
        {
            if (errno != EAGAIN && errno != ETIMEDOUT)
            {
                std::cerr << "VideoEncoder: capture_plane.dqBuffer failed errno=" << errno << std::endl;
            }
            return false;
        }

        if (!buf)
            return false;

        // Empty capture buffer (EOS / drain) - requeue and report no packet.
        if (buf->planes[0].bytesused == 0)
        {
            encoder_->capture_plane.qBuffer(v4l2_buf, nullptr);
            return false;
        }

        // Capture plane timestamp is unreliable under multi-slice (NVENC
        // zeroes it for non-first slice AUs). Use the queue: the first AU of
        // a frame pops the submit ts, subsequent slice AUs of the same frame
        // reuse |last_capture_ts_|. Single-slice path is also covered.
        uint64_t ts = 0;
        {
            std::lock_guard<std::mutex> lk(ts_queue_mutex_);
            if (!timestamp_queue_.empty())
            {
                ts = timestamp_queue_.front();
                timestamp_queue_.pop();
                last_capture_ts_ = ts;
            }
            else
            {
                ts = last_capture_ts_;
            }
        }

        const uint8_t *data = buf->planes[0].data;
        const size_t   size = buf->planes[0].bytesused;

        maybePrintSpsPpsOnce(data, size);
        parseSpsPps_(data, size);

        const bool keyframe = (config_.codec == Codec::kH265)
                                  ? containsH265IdrNal(data, size)
                                  : containsH264IdrNal(data, size);

        out.data           = data;
        out.size           = size;
        out.timestamp_us   = ts;
        out.keyframe       = keyframe;
        out._cap_buf_index = v4l2_buf.index;
        out._nvbuf         = buf;
        return true;
    }

    void VideoEncoder::releaseCapture(EncodedPacket &pkt)
    {
        if (!encoder_ || !pkt._nvbuf)
            return;

        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane  planes[MAX_PLANES];
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));
        v4l2_buf.index    = pkt._cap_buf_index;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        v4l2_buf.memory   = V4L2_MEMORY_MMAP;
        v4l2_buf.length   = encoder_->capture_plane.getNumPlanes();

        encoder_->capture_plane.qBuffer(v4l2_buf, nullptr);

        pkt.data   = nullptr;
        pkt.size   = 0;
        pkt._nvbuf = nullptr;
    }

    void VideoEncoder::parseSpsPps_(const uint8_t *data, size_t size)
    {
        if (sps_pps_sent_.load(std::memory_order_relaxed))
            return;

        std::lock_guard<std::mutex> lk(sps_pps_mutex_);
        if (sps_pps_sent_.load(std::memory_order_relaxed))
            return;

        const H264StreamFormat fmt = detectH264Format(data, size);
        const bool is_h265 = (config_.codec == Codec::kH265);
        forEachNal(data, size, fmt, [this, is_h265](const uint8_t *nal, size_t nal_size) {
            if (is_h265)
            {
                if (nal_size < 2)
                    return;
                // HEVC: 2-byte NAL header, type = bits 1..6 of byte 0.
                const uint8_t nal_unit_type = static_cast<uint8_t>((nal[0] >> 1) & 0x3Fu);
                if (nal_unit_type == 32 && cached_vps_.empty())  // VPS
                {
                    cached_vps_.assign(nal, nal + nal_size);
                }
                else if (nal_unit_type == 33 && cached_sps_.empty())  // SPS
                {
                    cached_sps_.assign(nal, nal + nal_size);
                }
                else if (nal_unit_type == 34 && cached_pps_.empty())  // PPS
                {
                    cached_pps_.assign(nal, nal + nal_size);
                }
            }
            else
            {
                if (nal_size < 1)
                    return;
                const uint8_t nal_unit_type = static_cast<uint8_t>(nal[0] & 0x1Fu);
                if (nal_unit_type == 7 && cached_sps_.empty())
                {
                    cached_sps_.assign(nal, nal + nal_size);
                }
                else if (nal_unit_type == 8 && cached_pps_.empty())
                {
                    cached_pps_.assign(nal, nal + nal_size);
                }
            }
        });

        const bool h265_ready = is_h265 && !cached_vps_.empty() && !cached_sps_.empty() && !cached_pps_.empty();
        const bool h264_ready = !is_h265 && !cached_sps_.empty() && !cached_pps_.empty();
        if (h265_ready || h264_ready)
        {
            sps_pps_sent_.store(true, std::memory_order_relaxed);
            if (sps_pps_callback_)
            {
                sps_pps_callback_(cached_sps_, cached_pps_, cached_vps_);
            }
        }
    }

    bool VideoEncoder::setBitrate(uint32_t bitrate_bps)
    {
        if (!encoder_ || bitrate_bps == 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lk(bitrate_mutex_);
        if (encoder_->setBitrate(bitrate_bps) < 0)
        {
            return false;
        }
        config_.bitrate = bitrate_bps;
        return true;
    }

} // namespace trb::video
