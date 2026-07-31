// serial_port.cpp — POSIX serial implementation.

#include "msd_arduino/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

namespace msd_arduino
{

namespace
{
speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    default:     return B115200;
  }
}
}  // namespace

SerialPort::SerialPort()
: fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close_port();
}

bool SerialPort::open_port(const std::string & path, int baud, int post_open_delay_ms)
{
  close_port();

  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    last_error_ = std::string("open(") + path + "): " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    last_error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close_port();
    return false;
  }

  // Raw 8N1, no flow control.
  cfmakeraw(&tty);
  const speed_t spd = baud_to_speed(baud);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  // Don't drop DTR on close: prevents an Arduino auto-reset on every reopen.
  tty.c_cflag &= ~HUPCL;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    last_error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close_port();
    return false;
  }

  // Most Arduino boards reset on the DTR rising edge that happens at open().
  // Wait for the bootloader to time out, then flush bootloader noise.
  if (post_open_delay_ms > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(post_open_delay_ms));
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialPort::read_some(uint8_t * buf, std::size_t max_len)
{
  if (fd_ < 0) {
    return -1;
  }
  ssize_t n = ::read(fd_, buf, max_len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    last_error_ = std::string("read: ") + std::strerror(errno);
    return -1;
  }
  return static_cast<int>(n);
}

bool SerialPort::write_all(const uint8_t * buf, std::size_t len)
{
  if (fd_ < 0) {
    return false;
  }
  std::size_t off = 0;
  while (off < len) {
    ssize_t n = ::write(fd_, buf + off, len - off);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      last_error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    off += static_cast<std::size_t>(n);
  }
  return true;
}

}  // namespace msd_arduino
