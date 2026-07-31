// serial_port.hpp — POSIX serial wrapper used by MsdSerialLink.
//
// Thin RAII wrapper around a tty fd configured for raw 8N1 binary I/O.
// Handles the Arduino USB-CDC quirk: opening /dev/ttyACM* on Linux pulses
// DTR which auto-resets the Mega; open_port() therefore sleeps for
// `post_open_delay_ms` (default 2000 ms) and flushes the input buffer
// once the bootloader has finished.

#ifndef MSD_ARDUINO__SERIAL_PORT_HPP_
#define MSD_ARDUINO__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace msd_arduino
{

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Open and configure the port. Returns false on failure (call
  // last_error() for details). post_open_delay_ms is the time to wait
  // after configuring the line so the Arduino bootloader can finish.
  bool open_port(const std::string & path, int baud, int post_open_delay_ms = 2000);

  void close_port();
  bool is_open() const {return fd_ >= 0;}

  // Non-blocking read. Returns bytes read (0 = none available, -1 = error).
  int read_some(uint8_t * buf, std::size_t max_len);

  // Write all bytes; retries on partial writes / EAGAIN.
  bool write_all(const uint8_t * buf, std::size_t len);

  const std::string & last_error() const {return last_error_;}

private:
  int         fd_;
  std::string last_error_;
};

}  // namespace msd_arduino

#endif  // MSD_ARDUINO__SERIAL_PORT_HPP_
