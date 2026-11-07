#pragma once

#include <cstddef>
#include <cstdint>

namespace trb::udp::gf256 {

uint8_t mul(uint8_t a, uint8_t b);
uint8_t inv(uint8_t a);
uint8_t pow(uint8_t a, uint16_t e);
void mulAdd(uint8_t coef, const uint8_t *src, uint8_t *dst, size_t bytes);

}  // namespace trb::udp::gf256
