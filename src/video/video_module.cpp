#include "video/video_module.hpp"

#include <algorithm>
#include <utility>

namespace trb::video {

VideoModule::Config VideoModule::Config::configFromRosParam(ros::NodeHandle& nh) {
  Config c;
  std::vector<int> profile;
  nh.param<std::vector<int>>("video/profile", profile, {3840, 1520, 30});
  if (profile.size() == 3 && profile[0] > 0 && profile[1] > 0 && profile[2] > 0) {
    c.width = static_cast<uint32_t>(profile[0]);
    c.height = static_cast<uint32_t>(profile[1]);
    c.fps = static_cast<uint32_t>(profile[2]);
  }
  std::string codec_str;
  nh.param<std::string>("video/encoder/codec", codec_str, "h264");
  std::transform(codec_str.begin(), codec_str.end(), codec_str.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (codec_str == "h265" || codec_str == "hevc") {
    c.codec = Config::Codec::kH265;
  } else if (codec_str == "h264" || codec_str == "avc") {
    c.codec = Config::Codec::kH264;
  } else {
    ROS_WARN("Unknown video/encoder/codec='%s' (use h264|h265). Falling back to h264.",
             codec_str.c_str());
    c.codec = Config::Codec::kH264;
  }
  return c;
}

VideoModule::VideoModule(ros::NodeHandle nh, Config config)
    : nh_(std::move(nh)), config_(std::move(config)) {
#ifdef JETSON_MM_API_ENABLED
  manager_ = std::make_unique<VideoStreamManager>(nh_);
  manager_->setEncodedFrameCallback(
      [this](const uint8_t* data, size_t size, uint64_t ts, bool kf) {
        if (encoded_cb_) encoded_cb_(data, size, ts, kf);
      });
  manager_->setSpsPpsCallback(
      [this](const std::vector<uint8_t>& sps, const std::vector<uint8_t>& pps,
             const std::vector<uint8_t>& vps) {
        {
          std::lock_guard<std::mutex> lock(sps_pps_mutex_);
          cached_sps_ = sps;
          cached_pps_ = pps;
          cached_vps_ = vps;
        }
        if (sps_pps_cb_) sps_pps_cb_(sps, pps, vps);
      });
  ROS_INFO("VideoModule: created (profile %ux%u@%u)",
           config_.width, config_.height, config_.fps);
#else
  ROS_INFO("VideoModule disabled (Jetson MM API not available)");
#endif
}

VideoModule::~VideoModule() { stop(); }

bool VideoModule::start() {
#ifdef JETSON_MM_API_ENABLED
  if (!manager_) return false;
  if (manager_->isRunning()) {
    // Already running; if SPS/PPS was cached before unpair, re-emit so the
    // state machine can rebuild VideoConfig for the new peer.
    std::vector<uint8_t> sps, pps, vps;
    {
      std::lock_guard<std::mutex> lock(sps_pps_mutex_);
      sps = cached_sps_;
      pps = cached_pps_;
      vps = cached_vps_;
    }
    if (!sps.empty() && !pps.empty() && sps_pps_cb_) {
      sps_pps_cb_(sps, pps, vps);
    }
    return true;
  }
  ROS_INFO("VideoModule: starting VideoStreamManager...");
  if (!manager_->start()) {
    ROS_WARN("VideoModule: VideoStreamManager start failed");
    return false;
  }
  ROS_INFO("VideoModule: VideoStreamManager started");
  return true;
#else
  return false;
#endif
}

void VideoModule::stop() {
#ifdef JETSON_MM_API_ENABLED
  if (manager_) {
    manager_->stop();
  }
#endif
}

bool VideoModule::isRunning() const {
#ifdef JETSON_MM_API_ENABLED
  return manager_ && manager_->isRunning();
#else
  return false;
#endif
}

void VideoModule::setEncodedFrameCallback(EncodedFrameCallback cb) {
  encoded_cb_ = std::move(cb);
}

void VideoModule::setSpsPpsCallback(SpsPpsCallback cb) {
  sps_pps_cb_ = std::move(cb);
}

bool VideoModule::hasSpsPps() const {
  std::lock_guard<std::mutex> lock(sps_pps_mutex_);
  return !cached_sps_.empty() && !cached_pps_.empty();
}

bool VideoModule::getSpsPps(std::vector<uint8_t>& sps,
                            std::vector<uint8_t>& pps,
                            std::vector<uint8_t>& vps) const {
  std::lock_guard<std::mutex> lock(sps_pps_mutex_);
  if (cached_sps_.empty() || cached_pps_.empty()) return false;
  sps = cached_sps_;
  pps = cached_pps_;
  vps = cached_vps_;
  return true;
}

void VideoModule::clearSpsPps() {
  std::lock_guard<std::mutex> lock(sps_pps_mutex_);
  cached_sps_.clear();
  cached_pps_.clear();
  cached_vps_.clear();
}

VideoModule::StatsSnapshot VideoModule::consumeStats() {
#ifdef JETSON_MM_API_ENABLED
  if (!manager_) return StatsSnapshot{};
  return manager_->consumeStats();
#else
  return StatsSnapshot{};
#endif
}

}  // namespace trb::video
