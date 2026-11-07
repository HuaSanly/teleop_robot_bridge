#include "udp/udp_socket.hpp"

#include <arpa/inet.h>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

namespace trb::udp {

UdpSocket::UdpSocket(const std::string &remote_ip, int remote_port, const std::string &bind_ip, int bind_port)
    : remote_ip_(remote_ip), remote_port_(remote_port), bind_ip_(bind_ip), bind_port_(bind_port) {
  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
}

UdpSocket::~UdpSocket() {
  close();
}

bool UdpSocket::open() {
  if (sockfd_ >= 0) {
    return true;
  }

  sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    return false;
  }

  if (!bind_ip_.empty() || bind_port_ > 0) {
    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(bind_port_));
    if (bind_ip_.empty()) {
      addr.sin_addr.s_addr = INADDR_ANY;
    } else if (inet_pton(AF_INET, bind_ip_.c_str(), &addr.sin_addr) <= 0) {
      close();
      return false;
    }

    if (::bind(sockfd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      close();
      return false;
    }
  }

  if (!resetRemote()) {
    close();
    return false;
  }
  return true;
}

void UdpSocket::close() {
  if (sockfd_ >= 0) {
    ::close(sockfd_);
    sockfd_ = -1;
  }
}

void UdpSocket::setRemote(const std::string &remote_ip, int remote_port) {
  remote_ip_ = remote_ip;
  remote_port_ = remote_port;
  (void)resetRemote();
}

void UdpSocket::setSendNonBlocking(bool enabled) {
  send_nonblocking_ = enabled;
}

void UdpSocket::setSendBufferBytes(int bytes) {
  if (sockfd_ < 0 || bytes <= 0) {
    return;
  }
  ::setsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &bytes, sizeof(bytes));
}

int UdpSocket::sendBufferBytes() const {
  if (sockfd_ < 0) {
    return -1;
  }

  int bytes = 0;
  socklen_t len = sizeof(bytes);
  if (::getsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &bytes, &len) < 0) {
    return -1;
  }
  return bytes;
}

void UdpSocket::setRecvTimeoutMs(int timeout_ms) {
  recv_timeout_ms_ = timeout_ms;
  if (sockfd_ < 0) {
    return;
  }
  timeval tv;
  tv.tv_sec = timeout_ms > 0 ? timeout_ms / 1000 : 0;
  tv.tv_usec = timeout_ms > 0 ? (timeout_ms % 1000) * 1000 : 0;
  // Always write SO_RCVTIMEO so that switching from a timeout to zero (blocking) works too.
  ::setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&tv), sizeof(tv));
}

int UdpSocket::send(const void *data, size_t size) {
  if (sockfd_ < 0) {
    return -1;
  }
  const int flags = send_nonblocking_ ? MSG_DONTWAIT : 0;
  return ::sendto(sockfd_, data, size, flags, reinterpret_cast<sockaddr *>(&remote_addr_), sizeof(remote_addr_));
}

int UdpSocket::receive(void *buffer, size_t size) {
  if (sockfd_ < 0) {
    return -1;
  }

  // The recv timeout is configured once in setRecvTimeoutMs (and whenever
  // open() is followed by UdpManager::start()'s explicit call); avoid
  // re-issuing setsockopt on every receive.
  sockaddr_in src_addr;
  socklen_t addr_len = sizeof(src_addr);
  return ::recvfrom(sockfd_, buffer, size, 0, reinterpret_cast<sockaddr *>(&src_addr), &addr_len);
}

bool UdpSocket::resetRemote() {
  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port = htons(static_cast<uint16_t>(remote_port_));
  if (remote_ip_.empty()) {
    remote_addr_.sin_addr.s_addr = INADDR_ANY;
    return true;
  }
  if (inet_pton(AF_INET, remote_ip_.c_str(), &remote_addr_.sin_addr) == 1) {
    return true;
  }

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo *result = nullptr;
  if (::getaddrinfo(remote_ip_.c_str(), nullptr, &hints, &result) != 0 ||
      result == nullptr) {
    if (result) ::freeaddrinfo(result);
    return false;
  }
  remote_addr_.sin_addr =
      reinterpret_cast<sockaddr_in *>(result->ai_addr)->sin_addr;
  ::freeaddrinfo(result);
  return true;
}

}  // namespace trb::udp
