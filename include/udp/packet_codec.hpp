#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace trb::udp {

constexpr uint8_t kVideoPacketType = 0x01;
constexpr uint8_t kPosePacketType = 0x02;
constexpr uint8_t kAudioPacketType = 0x04;

struct VideoPacketHeaderV2 {
  uint8_t Type{0};
  uint16_t PacketSeqNum{0};
  uint64_t Timestamp{0};
  uint32_t FrameId{0};
  uint16_t FragmentIndex{0};
  uint16_t TotalFragments{0};
  uint16_t PayloadLength{0};
  uint32_t FramePayloadLength{0};
  uint8_t FecTableId{0};
  uint8_t KeyframeFlag{0};
};

struct AudioPacketHeaderV2 {
  uint8_t Type{0};
  uint16_t Seq{0};
  uint64_t TimestampNtp{0};
};

class PacketCodec {
 public:
  explicit PacketCodec(size_t max_payload_bytes);

  void setSessionId(const std::string &session_id);
  size_t maxPayloadBytes() const;

  std::string buildHelloMessage() const;
  std::string buildPingMessage() const;

  void buildVideoPacket(const VideoPacketHeaderV2 &header,
                        const uint8_t *payload,
                        size_t payload_bytes,
                        std::vector<uint8_t> &out) const;
  void buildAudioPacket(const AudioPacketHeaderV2 &header,
                        const uint8_t *payload,
                        size_t payload_bytes,
                        std::vector<uint8_t> &out) const;

 private:
  std::string generateSignature(const std::string &type, const std::string &timestamp) const;

  size_t max_payload_bytes_{1200};
  std::string session_id_;
};

}  // namespace trb::udp
