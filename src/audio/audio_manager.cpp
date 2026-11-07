#include "audio/audio_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <sstream>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <ros/ros.h>

namespace trb::audio {

namespace {

std::mutex &gstInitMutex() {
  static std::mutex mutex;
  return mutex;
}

bool ensureGstInitialized() {
  std::lock_guard<std::mutex> lock(gstInitMutex());
  if (gst_is_initialized()) {
    return true;
  }

  GError *error = nullptr;
  if (!gst_init_check(nullptr, nullptr, &error)) {
    const char *message = (error && error->message) ? error->message : "unknown GStreamer init error";
    ROS_ERROR("GStreamer init failed: %s", message);
    if (error) {
      g_error_free(error);
    }
    return false;
  }
  return true;
}

uint64_t currentNtpTimestamp() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds);

  const uint32_t ntp_seconds = static_cast<uint32_t>(seconds.count() + 2208988800ULL);
  const uint32_t ntp_fraction = static_cast<uint32_t>((static_cast<long double>(nanoseconds.count()) * 4294967296.0L) /
                                                      1000000000.0L);
  return (static_cast<uint64_t>(ntp_seconds) << 32) | ntp_fraction;
}

uint64_t addNsToNtp(uint64_t base_ntp, uint64_t delta_ns) {
  const uint64_t ntp_seconds = base_ntp >> 32;
  const uint64_t ntp_fraction = base_ntp & 0xFFFFFFFFULL;

  const uint64_t add_seconds = delta_ns / 1000000000ULL;
  const uint64_t rem_ns = delta_ns % 1000000000ULL;
  const uint64_t add_fraction = static_cast<uint64_t>((static_cast<long double>(rem_ns) * 4294967296.0L) /
                                                       1000000000.0L);

  uint64_t fraction = ntp_fraction + add_fraction;
  uint64_t seconds = ntp_seconds + add_seconds;
  if (fraction >= (1ULL << 32)) {
    fraction -= (1ULL << 32);
    ++seconds;
  }
  return (seconds << 32) | fraction;
}

uint64_t ntpDiffToNs(uint64_t earlier_ntp, uint64_t later_ntp) {
  if (later_ntp <= earlier_ntp) {
    return 0;
  }

  const uint64_t earlier_seconds = earlier_ntp >> 32;
  const uint64_t earlier_fraction = earlier_ntp & 0xFFFFFFFFULL;
  const uint64_t later_seconds = later_ntp >> 32;
  const uint64_t later_fraction = later_ntp & 0xFFFFFFFFULL;

  uint64_t seconds_diff = later_seconds - earlier_seconds;
  int64_t fraction_diff = static_cast<int64_t>(later_fraction) - static_cast<int64_t>(earlier_fraction);
  if (fraction_diff < 0) {
    --seconds_diff;
    fraction_diff += static_cast<int64_t>(1ULL << 32);
  }

  const uint64_t ns_from_seconds = seconds_diff * 1000000000ULL;
  const uint64_t ns_from_fraction = static_cast<uint64_t>((static_cast<long double>(fraction_diff) * 1000000000.0L) /
                                                           4294967296.0L);
  return ns_from_seconds + ns_from_fraction;
}

}  // namespace

struct AudioManager::Impl {
  Config config;
  std::atomic<bool> initialized{false};
  std::atomic<bool> running{false};
  std::atomic<bool> capture_active{false};
  std::atomic<bool> playback_active{false};
  std::string capture_pipeline_description;
  std::string playback_pipeline_description;

  GstElement *capture_pipeline{nullptr};
  GstElement *playback_pipeline{nullptr};
  GstAppSink *capture_sink{nullptr};
  GstAppSrc *playback_src{nullptr};

  std::mutex callback_mutex;
  EncodedPacketCallback encoded_packet_callback;

  std::mutex playback_mutex;
  bool capture_pts_initialized{false};
  uint64_t capture_base_ntp{0};
  uint64_t capture_first_pts_ns{0};
  uint64_t capture_packet_index{0};

  bool playback_pts_initialized{false};
  uint64_t playback_base_ntp{0};
  uint64_t playback_last_pts_ns{0};

  std::mutex stats_mutex;
  std::chrono::steady_clock::time_point stats_last_log{std::chrono::steady_clock::time_point::min()};
  uint64_t capture_packets_window{0};
  uint64_t capture_bytes_window{0};
  uint64_t uplink_packets_window{0};
  uint64_t uplink_bytes_window{0};
  uint64_t downlink_packets_window{0};
  uint64_t downlink_bytes_window{0};
  uint64_t playback_push_failures_window{0};

  void maybeLogStatsLocked(const std::chrono::steady_clock::time_point &now) {
    if (stats_last_log == std::chrono::steady_clock::time_point::min()) {
      stats_last_log = now;
      return;
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_last_log).count();
    if (elapsed_ms < 1000) {
      return;
    }

    const bool has_activity = capture_packets_window > 0 || uplink_packets_window > 0 ||
                              downlink_packets_window > 0 || playback_push_failures_window > 0;
    if (!has_activity) {
      stats_last_log = now;
      return;
    }

    const double elapsed_sec = static_cast<double>(elapsed_ms) / 1000.0;
    const double uplink_pps = static_cast<double>(uplink_packets_window) / elapsed_sec;
    const double uplink_kbps = static_cast<double>(uplink_bytes_window) * 8.0 / static_cast<double>(elapsed_ms);
    const double downlink_pps = static_cast<double>(downlink_packets_window) / elapsed_sec;
    const double downlink_kbps = static_cast<double>(downlink_bytes_window) * 8.0 / static_cast<double>(elapsed_ms);
    const double capture_pps = static_cast<double>(capture_packets_window) / elapsed_sec;

    ROS_INFO("[AUDIO] cap=%.1fpps up=%.1fpps %.1fkbps down=%.1fpps %.1fkbps push_fail=%lu running=%d",
             capture_pps,
             uplink_pps,
             uplink_kbps,
             downlink_pps,
             downlink_kbps,
             static_cast<unsigned long>(playback_push_failures_window),
             running.load() ? 1 : 0);

    capture_packets_window = 0;
    capture_bytes_window = 0;
    uplink_packets_window = 0;
    uplink_bytes_window = 0;
    downlink_packets_window = 0;
    downlink_bytes_window = 0;
    playback_push_failures_window = 0;
    stats_last_log = now;
  }

  void noteCapturePacket(size_t size) {
    std::lock_guard<std::mutex> lock(stats_mutex);
    ++capture_packets_window;
    capture_bytes_window += size;
    maybeLogStatsLocked(std::chrono::steady_clock::now());
  }

  void noteUplinkPacket(size_t size) {
    std::lock_guard<std::mutex> lock(stats_mutex);
    ++uplink_packets_window;
    uplink_bytes_window += size;
    maybeLogStatsLocked(std::chrono::steady_clock::now());
  }

  void noteDownlinkPacket(size_t size, bool push_ok) {
    std::lock_guard<std::mutex> lock(stats_mutex);
    ++downlink_packets_window;
    downlink_bytes_window += size;
    if (!push_ok) {
      ++playback_push_failures_window;
    }
    maybeLogStatsLocked(std::chrono::steady_clock::now());
  }

  static GstFlowReturn onNewSample(GstAppSink *sink, gpointer user_data) {
    auto *self = static_cast<Impl *>(user_data);
    return self ? self->handleNewSample(sink) : GST_FLOW_ERROR;
  }

  GstFlowReturn handleNewSample(GstAppSink *sink) {
    if (!running.load() || !capture_active.load()) {
      return GST_FLOW_FLUSHING;
    }

    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
      return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    uint64_t pts_ns = GST_BUFFER_PTS_IS_VALID(buffer) ? GST_BUFFER_PTS(buffer) : GST_CLOCK_TIME_NONE;
    const uint64_t frame_duration_ns = static_cast<uint64_t>(config.frame_duration_ms) * GST_MSECOND;
    if (!capture_pts_initialized) {
      capture_pts_initialized = true;
      capture_base_ntp = currentNtpTimestamp();
      capture_first_pts_ns = (pts_ns == GST_CLOCK_TIME_NONE) ? 0 : pts_ns;
      capture_packet_index = 0;
    }

    uint64_t relative_ns = 0;
    if (pts_ns == GST_CLOCK_TIME_NONE) {
      relative_ns = capture_packet_index * frame_duration_ns;
    } else if (pts_ns >= capture_first_pts_ns) {
      relative_ns = pts_ns - capture_first_pts_ns;
    }
    const uint64_t timestamp_ntp = addNsToNtp(capture_base_ntp, relative_ns);
    ++capture_packet_index;
    noteCapturePacket(map_info.size);

    EncodedPacketCallback callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex);
      callback = encoded_packet_callback;
    }
    if (callback) {
      noteUplinkPacket(map_info.size);
      callback(map_info.data, map_info.size, timestamp_ntp);
    }

    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  static void unrefElement(GstElement *&element) {
    if (element) {
      gst_object_unref(element);
      element = nullptr;
    }
  }

  static void unrefAppSink(GstAppSink *&sink) {
    if (sink) {
      gst_object_unref(sink);
      sink = nullptr;
    }
  }

  static void unrefAppSrc(GstAppSrc *&src) {
    if (src) {
      gst_object_unref(src);
      src = nullptr;
    }
  }

  static void logPipelineMessages(GstElement *pipeline, const char *pipeline_name, const std::string &description) {
    if (!pipeline) {
      return;
    }

    GstBus *bus = gst_element_get_bus(pipeline);
    if (!bus) {
      return;
    }

    ROS_WARN("%s description: %s", pipeline_name, description.c_str());
    while (true) {
      GstMessage *msg = gst_bus_pop_filtered(
          bus,
          static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_INFO));
      if (!msg) {
        break;
      }

      switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
          GError *error = nullptr;
          gchar *debug = nullptr;
          gst_message_parse_error(msg, &error, &debug);
          ROS_ERROR("%s bus error from %s: %s%s%s",
                    pipeline_name,
                    GST_OBJECT_NAME(msg->src),
                    error && error->message ? error->message : "unknown",
                    debug ? " | debug=" : "",
                    debug ? debug : "");
          if (error) {
            g_error_free(error);
          }
          if (debug) {
            g_free(debug);
          }
          break;
        }
        case GST_MESSAGE_WARNING: {
          GError *error = nullptr;
          gchar *debug = nullptr;
          gst_message_parse_warning(msg, &error, &debug);
          ROS_WARN("%s bus warning from %s: %s%s%s",
                   pipeline_name,
                   GST_OBJECT_NAME(msg->src),
                   error && error->message ? error->message : "unknown",
                   debug ? " | debug=" : "",
                   debug ? debug : "");
          if (error) {
            g_error_free(error);
          }
          if (debug) {
            g_free(debug);
          }
          break;
        }
        case GST_MESSAGE_INFO: {
          GError *error = nullptr;
          gchar *debug = nullptr;
          gst_message_parse_info(msg, &error, &debug);
          ROS_INFO("%s bus info from %s: %s%s%s",
                   pipeline_name,
                   GST_OBJECT_NAME(msg->src),
                   error && error->message ? error->message : "unknown",
                   debug ? " | debug=" : "",
                   debug ? debug : "");
          if (error) {
            g_error_free(error);
          }
          if (debug) {
            g_free(debug);
          }
          break;
        }
        default:
          break;
      }

      gst_message_unref(msg);
    }

    gst_object_unref(bus);
  }

  void teardown() {
    stopPipelines();
    unrefAppSink(capture_sink);
    unrefAppSrc(playback_src);
    unrefElement(capture_pipeline);
    unrefElement(playback_pipeline);
    initialized.store(false);
  }

  void stopPipelines() {
    running.store(false);
    capture_active.store(false);
    playback_active.store(false);
    if (capture_pipeline) {
      gst_element_set_state(capture_pipeline, GST_STATE_NULL);
    }
    if (playback_pipeline) {
      gst_element_set_state(playback_pipeline, GST_STATE_NULL);
    }
    capture_pts_initialized = false;
    playback_pts_initialized = false;
    capture_base_ntp = 0;
    playback_base_ntp = 0;
    capture_first_pts_ns = 0;
    playback_last_pts_ns = 0;
    capture_packet_index = 0;
  }

  std::string buildCapturePipelineDescription() const {
    std::ostringstream pipeline;
    pipeline << "alsasrc name=audio_capture_src do-timestamp=true";
    if (!config.capture_device.empty()) {
      pipeline << " device=" << config.capture_device;
    }
    pipeline << " ! audioconvert"
             << " ! audioresample"
             << " ! audio/x-raw,format=S16LE,rate=" << config.sample_rate
             << ",channels=" << config.channels;
    if (config.aec_enabled && config.downlink_enabled) {
      pipeline << " ! webrtcdsp name=audio_dsp echo-cancel=true noise-suppression=true gain-control=true"
               << " extended-filter=true probe=audio_echo_probe";
    }
    pipeline << " ! queue max-size-buffers=4 leaky=downstream"
             << " ! opusenc audio-type=voice frame-size=" << config.frame_duration_ms
             << " bitrate=" << config.bitrate_bps
             << " inband-fec=" << (config.opus_inband_fec_enabled ? "true" : "false")
             << " dtx=" << (config.opus_dtx_enabled ? "true" : "false")
             << " ! appsink name=audio_capture_sink emit-signals=true sync=false max-buffers=4 drop=true";
    return pipeline.str();
  }

  std::string buildPlaybackPipelineDescription() const {
    std::ostringstream pipeline;
    pipeline << "appsrc name=audio_playback_src is-live=true format=time block=false do-timestamp=false"
             << " caps=audio/x-opus,channel-mapping-family=(int)0,channels=(int)" << config.channels
             << ",rate=(int)" << config.sample_rate
             << " ! queue max-size-buffers=8 leaky=downstream"
             << " ! opusparse"
             << " ! opusdec"
             << " ! audioconvert"
             << " ! audioresample"
             << " ! audio/x-raw,format=S16LE,rate=" << config.sample_rate
             << ",channels=" << config.channels;
    if (config.aec_enabled && config.uplink_enabled) {
      pipeline << " ! webrtcechoprobe name=audio_echo_probe";
    }
    if (config.playback_device.empty()) {
      pipeline << " ! autoaudiosink name=audio_playback_sink sync=true";
    } else {
      pipeline << " ! alsasink name=audio_playback_sink sync=true device=" << config.playback_device;
    }
    return pipeline.str();
  }

  bool createCapturePipeline() {
    GError *error = nullptr;
    capture_pipeline_description = buildCapturePipelineDescription();
    ROS_INFO("Audio capture pipeline: %s", capture_pipeline_description.c_str());
    capture_pipeline = gst_parse_launch(capture_pipeline_description.c_str(), &error);
    if (!capture_pipeline) {
      const char *message = (error && error->message) ? error->message : "unknown capture pipeline error";
      ROS_ERROR("Failed to create audio capture pipeline: %s", message);
      if (error) {
        g_error_free(error);
      }
      return false;
    }
    if (error) {
      ROS_WARN("Audio capture pipeline warning: %s", error->message);
      g_error_free(error);
    }

    capture_sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(capture_pipeline), "audio_capture_sink"));
    if (!capture_sink) {
      ROS_ERROR("Failed to get appsink from audio capture pipeline");
      return false;
    }

    g_signal_connect(capture_sink, "new-sample", G_CALLBACK(&Impl::onNewSample), this);
    return true;
  }

  bool createPlaybackPipeline() {
    GError *error = nullptr;
    playback_pipeline_description = buildPlaybackPipelineDescription();
    ROS_INFO("Audio playback pipeline: %s", playback_pipeline_description.c_str());
    playback_pipeline = gst_parse_launch(playback_pipeline_description.c_str(), &error);
    if (!playback_pipeline) {
      const char *message = (error && error->message) ? error->message : "unknown playback pipeline error";
      ROS_ERROR("Failed to create audio playback pipeline: %s", message);
      if (error) {
        g_error_free(error);
      }
      return false;
    }
    if (error) {
      ROS_WARN("Audio playback pipeline warning: %s", error->message);
      g_error_free(error);
    }

    playback_src = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(playback_pipeline), "audio_playback_src"));
    if (!playback_src) {
      ROS_ERROR("Failed to get appsrc from audio playback pipeline");
      return false;
    }

    gst_app_src_set_stream_type(playback_src, GST_APP_STREAM_TYPE_STREAM);
    gst_app_src_set_max_bytes(playback_src, 512 * 1024);
    return true;
  }

  bool initializePipelines(const Config &cfg) {
    config = cfg;
    if (!ensureGstInitialized()) {
      return false;
    }

    teardown();
    config = cfg;
    if (config.enabled && config.uplink_enabled && !createCapturePipeline()) {
      teardown();
      return false;
    }
    if (config.enabled && config.downlink_enabled && !createPlaybackPipeline()) {
      teardown();
      return false;
    }

    initialized.store(true);
    return true;
  }

  bool startPipelines() {
    if (!config.enabled) {
      running.store(false);
      capture_active.store(false);
      playback_active.store(false);
      return true;
    }
    if (!initialized.load()) {
      return false;
    }

    capture_pts_initialized = false;
    playback_pts_initialized = false;
    capture_packet_index = 0;
    playback_last_pts_ns = 0;
    capture_active.store(false);
    playback_active.store(false);

    bool any_started = false;

    if (capture_pipeline) {
      const GstStateChangeReturn ret = gst_element_set_state(capture_pipeline, GST_STATE_PLAYING);
      if (ret == GST_STATE_CHANGE_FAILURE) {
        ROS_WARN("Failed to start audio capture pipeline, continuing without uplink audio");
        logPipelineMessages(capture_pipeline, "audio capture pipeline", capture_pipeline_description);
        gst_element_set_state(capture_pipeline, GST_STATE_NULL);
        unrefAppSink(capture_sink);
        unrefElement(capture_pipeline);
      } else {
        capture_active.store(true);
        any_started = true;
      }
    }
    if (playback_pipeline) {
      const GstStateChangeReturn ret = gst_element_set_state(playback_pipeline, GST_STATE_PLAYING);
      if (ret == GST_STATE_CHANGE_FAILURE) {
        ROS_WARN("Failed to start audio playback pipeline, continuing without downlink audio");
        logPipelineMessages(playback_pipeline, "audio playback pipeline", playback_pipeline_description);
        gst_element_set_state(playback_pipeline, GST_STATE_NULL);
        unrefAppSrc(playback_src);
        unrefElement(playback_pipeline);
      } else {
        playback_active.store(true);
        any_started = true;
      }
    }

    if (!any_started) {
      running.store(false);
      return false;
    }

    running.store(true);
    ROS_INFO("Audio pipelines started: uplink=%d downlink=%d",
             capture_active.load() ? 1 : 0,
             playback_active.load() ? 1 : 0);
    return true;
  }

  bool pushRemotePacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp) {
    if (!running.load() || !playback_active.load() || !playback_src || !data || size == 0) {
      return false;
    }

    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    if (!buffer) {
      return false;
    }

    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_WRITE)) {
      gst_buffer_unref(buffer);
      return false;
    }
    std::memcpy(map_info.data, data, size);
    gst_buffer_unmap(buffer, &map_info);

    const uint64_t duration_ns = static_cast<uint64_t>(config.frame_duration_ms) * GST_MSECOND;

    std::lock_guard<std::mutex> lock(playback_mutex);
    if (!playback_pts_initialized) {
      playback_pts_initialized = true;
      playback_base_ntp = timestamp_ntp;
      playback_last_pts_ns = 0;
    }

    uint64_t pts_ns = ntpDiffToNs(playback_base_ntp, timestamp_ntp);
    if (pts_ns < playback_last_pts_ns) {
      pts_ns = playback_last_pts_ns + duration_ns;
    }
    playback_last_pts_ns = pts_ns;

    GST_BUFFER_PTS(buffer) = pts_ns;
    GST_BUFFER_DTS(buffer) = pts_ns;
    GST_BUFFER_DURATION(buffer) = duration_ns;

    const GstFlowReturn ret = gst_app_src_push_buffer(playback_src, buffer);
    if (ret != GST_FLOW_OK) {
      noteDownlinkPacket(size, false);
      ROS_WARN_THROTTLE(1.0, "Failed to push remote Opus packet into playback pipeline: %d", ret);
      return false;
    }
    noteDownlinkPacket(size, true);
    return true;
  }
};

AudioManager::AudioManager() : impl_(new Impl()) {}

AudioManager::~AudioManager() {
  if (impl_) {
    impl_->teardown();
    delete impl_;
    impl_ = nullptr;
  }
}

bool AudioManager::initialize(const Config &config) { return impl_->initializePipelines(config); }

bool AudioManager::start() { return impl_->startPipelines(); }

void AudioManager::stop() { impl_->stopPipelines(); }

bool AudioManager::isRunning() const { return impl_->running.load(); }

void AudioManager::setEncodedPacketCallback(EncodedPacketCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->encoded_packet_callback = std::move(callback);
}

bool AudioManager::pushRemoteOpusPacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp) {
  return impl_->pushRemotePacket(data, size, timestamp_ntp);
}

const AudioManager::Config &AudioManager::config() const { return impl_->config; }

}  // namespace trb::audio