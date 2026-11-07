#include "udp/gf256.hpp"

#include <array>

namespace trb::udp::gf256 {

namespace {
constexpr uint16_t kPrimPoly = 0x11D;

struct Tables {
  std::array<uint8_t, 256> log{};
  std::array<uint8_t, 512> exp{};
  bool ready{false};

  void init() {
    uint16_t x = 1;
    for (int i = 0; i < 255; ++i) {
      exp[i] = static_cast<uint8_t>(x);
      log[static_cast<uint8_t>(x)] = static_cast<uint8_t>(i);

      x <<= 1;
      if (x & 0x100) {
        x ^= kPrimPoly;
      }
    }

    for (int i = 255; i < 512; ++i) {
      exp[i] = exp[i - 255];
    }

    ready = true;
  }
};

Tables &tables() {
  static Tables t;
  if (!t.ready) {
    t.init();
  }
  return t;
}
}  // namespace

uint8_t mul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0) {
    return 0;
  }
  auto &t = tables();
  const uint16_t la = t.log[a];
  const uint16_t lb = t.log[b];
  return t.exp[la + lb];
}

uint8_t inv(uint8_t a) {
  if (a == 0) {
    return 0;
  }
  auto &t = tables();
  const uint16_t la = t.log[a];
  return t.exp[255u - la];
}

uint8_t pow(uint8_t a, uint16_t e) {
  if (e == 0) {
    return 1;
  }
  if (a == 0) {
    return 0;
  }

  auto &t = tables();
  const uint16_t la = t.log[a];
  const uint32_t idx = (static_cast<uint32_t>(la) * static_cast<uint32_t>(e)) % 255u;
  return t.exp[idx];
}

void mulAdd(uint8_t coef, const uint8_t *src, uint8_t *dst, size_t bytes) {
  if (coef == 0 || bytes == 0) {
    return;
  }

  if (coef == 1) {
    for (size_t i = 0; i < bytes; ++i) {
      dst[i] ^= src[i];
    }
    return;
  }

  auto &t = tables();
  const uint8_t lcoef = t.log[coef];

  for (size_t i = 0; i < bytes; ++i) {
    const uint8_t s = src[i];
    if (s == 0) {
      continue;
    }
    dst[i] ^= t.exp[lcoef + t.log[s]];
  }
}

}  // namespace trb::udp::gf256
