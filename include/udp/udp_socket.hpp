#pragma once

#include <cstddef>
#include <string>

#include <netinet/in.h>

namespace trb::udp {

class UdpSocket {
 public:
  UdpSocket(const std::string &remote_ip, int remote_port, const std::string &bind_ip, int bind_port);
  ~UdpSocket();

  bool open();
  void close();

  void setRemote(const std::string &remote_ip, int remote_port);
  void setSendNonBlocking(bool enabled);
  void setSendBufferBytes(int bytes);
  int sendBufferBytes() const;
  void setRecvTimeoutMs(int timeout_ms);

  int send(const void *data, size_t size);
  int receive(void *buffer, size_t size);

 private:
  bool resetRemote();

  std::string remote_ip_;
  int remote_port_{0};
  std::string bind_ip_;
  int bind_port_{0};
  int sockfd_{-1};
  bool send_nonblocking_{true};
  int recv_timeout_ms_{100};
  sockaddr_in remote_addr_{};
};

}  // namespace trb::udp
