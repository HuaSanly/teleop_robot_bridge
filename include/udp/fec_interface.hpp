#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace trb::udp {

class IFecEncoder {
 public:
  virtual ~IFecEncoder() = default;

  virtual bool getParams(uint8_t table_id, uint16_t total_fragments, uint8_t &groups, uint8_t &r_per_group) const = 0;

  virtual void buildParity(const uint8_t *data,
                           size_t size,
                           uint16_t total_fragments,
                           uint8_t groups,
                           uint8_t r_per_group,
                           size_t symbol_bytes,
                           std::vector<std::vector<uint8_t>> &out_parity) const = 0;
};

}  // namespace trb::udp
