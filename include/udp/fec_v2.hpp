#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "udp/fec_interface.hpp"

namespace trb::udp {

class FecV2 : public IFecEncoder {
 public:
  bool getParams(uint8_t table_id, uint16_t total_fragments, uint8_t &groups, uint8_t &r_per_group) const override;

  void buildParity(const uint8_t *data,
                   size_t size,
                   uint16_t total_fragments,
                   uint8_t groups,
                   uint8_t r_per_group,
                   size_t symbol_bytes,
                   std::vector<std::vector<uint8_t>> &out_parity) const override;

 private:
  struct V2FecTable {
    std::array<uint8_t, 256> groups_by_n{};
    std::array<uint8_t, 256> r_by_n{};
    bool loaded{false};
    std::string source_path;
  };

  bool loadTableOnce(V2FecTable &out) const;
  bool loadTableFromJsonText(const std::string &text, V2FecTable &out) const;
  bool loadTableFromJsonFile(const std::string &path, V2FecTable &out) const;

  const V2FecTable &table() const;
};

}  // namespace trb::udp
