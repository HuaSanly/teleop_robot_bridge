#include "udp/fec_v2.hpp"

#include "udp/gf256.hpp"

#include <algorithm>
#include <fstream>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>

namespace trb::udp {

namespace {
struct V2FecParams {
  uint8_t groups{0};
  uint8_t r{0};
};

bool parseIntAfterColon(const std::string &s, size_t &pos, int &out) {
  const size_t colon = s.find(':', pos);
  if (colon == std::string::npos) {
    return false;
  }
  size_t i = colon + 1;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) {
    ++i;
  }
  bool neg = false;
  if (i < s.size() && s[i] == '-') {
    neg = true;
    ++i;
  }
  if (i >= s.size() || s[i] < '0' || s[i] > '9') {
    return false;
  }
  long v = 0;
  while (i < s.size() && s[i] >= '0' && s[i] <= '9') {
    v = v * 10 + (s[i] - '0');
    ++i;
  }
  out = static_cast<int>(neg ? -v : v);
  pos = i;
  return true;
}
}  // namespace

bool FecV2::getParams(uint8_t table_id, uint16_t total_fragments, uint8_t &groups, uint8_t &r_per_group) const {
  groups = 0;
  r_per_group = 0;

  if (table_id == 0 || table_id != 1) {
    return false;
  }
  if (total_fragments >= 256) {
    return false;
  }

  const auto &t = table();
  if (!t.loaded) {
    return false;
  }

  groups = t.groups_by_n[static_cast<size_t>(total_fragments)];
  r_per_group = t.r_by_n[static_cast<size_t>(total_fragments)];
  return groups > 0 && r_per_group > 0;
}

void FecV2::buildParity(const uint8_t *data,
                        size_t size,
                        uint16_t total_fragments,
                        uint8_t groups,
                        uint8_t r_per_group,
                        size_t symbol_bytes,
                        std::vector<std::vector<uint8_t>> &out_parity) const {
  out_parity.clear();
  if (!data || size == 0 || groups == 0 || r_per_group == 0 || symbol_bytes == 0) {
    return;
  }

  // Cache Cauchy coefficients by (k,r)
  auto getCoef = [](uint8_t k, uint8_t r) -> const std::vector<uint8_t> & {
    struct Cache {
      std::mutex mu;
      std::unordered_map<uint32_t, std::vector<uint8_t>> m;
    };
    static Cache cache;

    const uint32_t key = (static_cast<uint32_t>(k) << 8) | static_cast<uint32_t>(r);
    {
      std::lock_guard<std::mutex> lk(cache.mu);
      auto it = cache.m.find(key);
      if (it != cache.m.end()) {
        return it->second;
      }
    }

    std::vector<uint8_t> coef;
    coef.resize(static_cast<size_t>(k) * static_cast<size_t>(r), 0);
    for (uint8_t j = 0; j < r; ++j) {
      for (uint8_t i = 0; i < k; ++i) {
        const uint8_t x = i;
        const uint8_t y = static_cast<uint8_t>(0x80u + static_cast<uint8_t>(j));
        coef[static_cast<size_t>(j) * static_cast<size_t>(k) + static_cast<size_t>(i)] =
            gf256::inv(static_cast<uint8_t>(x ^ y));
      }
    }

    std::lock_guard<std::mutex> lk(cache.mu);
    auto [it, _] = cache.m.emplace(key, std::move(coef));
    return it->second;
  };

  std::vector<uint8_t> k_by_group(groups, 0);
  std::vector<const std::vector<uint8_t> *> coef_by_group(groups, nullptr);

  for (uint8_t g = 0; g < groups; ++g) {
    uint8_t k_g = 0;
    for (uint32_t idx = g; idx < total_fragments; idx += groups) {
      ++k_g;
    }
    k_by_group[g] = k_g;
    if (k_g > 0 && r_per_group > 0) {
      coef_by_group[g] = &getCoef(k_g, r_per_group);
    }
  }

  out_parity.reserve(static_cast<size_t>(groups) * static_cast<size_t>(r_per_group));
  for (uint8_t p = 0; p < r_per_group; ++p) {
    for (uint8_t g = 0; g < groups; ++g) {
      const uint8_t k_g = k_by_group[g];
      if (k_g == 0) {
        out_parity.emplace_back();
        continue;
      }
      const auto *coef_ptr = coef_by_group[g];
      if (!coef_ptr) {
        out_parity.emplace_back();
        continue;
      }

      std::vector<uint8_t> parity(symbol_bytes, 0);
      const uint8_t *coef_row = coef_ptr->data() + static_cast<size_t>(p) * static_cast<size_t>(k_g);

      uint8_t local_i = 0;
      for (uint32_t src_idx = g; src_idx < total_fragments; src_idx += groups) {
        const size_t offset = static_cast<size_t>(src_idx) * symbol_bytes;
        const size_t chunk_size = std::min(symbol_bytes, size - offset);
        const uint8_t c = coef_row[local_i++];
        gf256::mulAdd(c, data + offset, parity.data(), chunk_size);
      }

      out_parity.emplace_back(std::move(parity));
    }
  }
}

bool FecV2::loadTableFromJsonText(const std::string &text, V2FecTable &out) const {
  out.groups_by_n.fill(0);
  out.r_by_n.fill(0);

  size_t pos = 0;
  int parsed = 0;
  while (true) {
    const size_t n_key = text.find("\"n\"", pos);
    if (n_key == std::string::npos) {
      break;
    }
    pos = n_key + 3;

    int n = 0;
    if (!parseIntAfterColon(text, pos, n)) {
      break;
    }

    const size_t g_key = text.find("\"groupCount\"", pos);
    if (g_key == std::string::npos) {
      break;
    }
    pos = g_key + 12;
    int group_count = 0;
    if (!parseIntAfterColon(text, pos, group_count)) {
      break;
    }

    const size_t p_key = text.find("\"parityCount\"", pos);
    if (p_key == std::string::npos) {
      break;
    }
    pos = p_key + 13;
    int parity_count = 0;
    if (!parseIntAfterColon(text, pos, parity_count)) {
      break;
    }

    if (n >= 0 && n < 256) {
      if (parity_count <= 0) {
        out.groups_by_n[static_cast<size_t>(n)] = 0;
        out.r_by_n[static_cast<size_t>(n)] = 0;
      } else {
        const int g = std::max(0, std::min(255, group_count));
        const int r = std::max(0, std::min(255, parity_count));
        out.groups_by_n[static_cast<size_t>(n)] = static_cast<uint8_t>(g);
        out.r_by_n[static_cast<size_t>(n)] = static_cast<uint8_t>(r);
      }
    }
    ++parsed;
  }

  if (parsed <= 0) {
    ROS_WARN("FEC table JSON parsed 0 entries; disabling FEC table");
    return false;
  }
  return true;
}

bool FecV2::loadTableFromJsonFile(const std::string &path, V2FecTable &out) const {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return false;
  }
  std::string text((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  if (text.empty()) {
    return false;
  }
  if (!loadTableFromJsonText(text, out)) {
    return false;
  }
  out.source_path = path;
  return true;
}

bool FecV2::loadTableOnce(V2FecTable &out) const {
  const std::string filename = "UDP_VIDEO_FEC_TABLE_V1.json";
  std::vector<std::string> candidates;
  candidates.emplace_back("config/" + filename);
  candidates.emplace_back("teleop_robot_bridge/config/" + filename);
  candidates.emplace_back("src/teleop_robot_bridge/config/" + filename);

  const std::string pkg_path = ros::package::getPath("teleop_robot_bridge");
  if (!pkg_path.empty()) {
    candidates.emplace_back(pkg_path + "/config/" + filename);
  }

  for (const auto &p : candidates) {
    if (loadTableFromJsonFile(p, out)) {
      out.loaded = true;
      return true;
    }
  }

  out.loaded = false;
  return false;
}

const FecV2::V2FecTable &FecV2::table() const {
  static V2FecTable table;
  static std::once_flag once;
  static std::atomic<bool> logged{false};

  std::call_once(once, [this]() { loadTableOnce(table); });

  if (!logged.exchange(true)) {
    if (table.loaded) {
      ROS_INFO("Loaded FEC table from %s", table.source_path.c_str());
    } else {
      ROS_WARN("Failed to load FEC table JSON; FEC will be disabled");
    }
  }

  return table;
}

}  // namespace trb::udp
