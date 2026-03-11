// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: POSIX serial port implementation for CRSF/ELRS

#include "evarobot_rc/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

// Define termios2 struct and ioctls manually to avoid header conflicts
// between <sys/ioctl.h> and <linux/termios.h>
#ifndef BOTHER
#define BOTHER 0010000
#endif

#ifndef TCGETS2
// aarch64 uses _IOR('T', 0x2A, ...) but we hardcode the value
#define TCGETS2 0x802C542A
#endif

#ifndef TCSETS2
#define TCSETS2 0x402C542B
#endif

struct termios2 {
  tcflag_t c_iflag;
  tcflag_t c_oflag;
  tcflag_t c_cflag;
  tcflag_t c_lflag;
  cc_t c_line;
  cc_t c_cc[19];
  speed_t c_ispeed;
  speed_t c_ospeed;
};

namespace evarobot_rc
{

SerialPort::SerialPort()
: fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baud_rate)
{
  // Close if already open
  if (fd_ >= 0) {
    close();
  }

  device_ = device;

  // Open serial port
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // Step 1: Initialize PL011 with standard termios at 460800 (closest standard rate)
  // This is required on Raspberry Pi 4B before termios2 BOTHER can set custom rates
  struct termios tty_std;
  std::memset(&tty_std, 0, sizeof(tty_std));
  tcgetattr(fd_, &tty_std);
  cfmakeraw(&tty_std);
  tty_std.c_cflag &= ~CRTSCTS;
  tty_std.c_cflag |= (CLOCAL | CREAD);
  tty_std.c_cflag &= ~CSIZE;
  tty_std.c_cflag |= CS8;
  tty_std.c_cflag &= ~PARENB;
  tty_std.c_cflag &= ~CSTOPB;
  tty_std.c_cc[VMIN] = 0;
  tty_std.c_cc[VTIME] = 0;
  cfsetispeed(&tty_std, B460800);
  cfsetospeed(&tty_std, B460800);
  tcsetattr(fd_, TCSANOW, &tty_std);

  // Step 2: Set exact baud rate via termios2 BOTHER (e.g. 420000 for CRSF)
  struct termios2 tty;
  std::memset(&tty, 0, sizeof(tty));

  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Set arbitrary baud rate using BOTHER flag
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = baud_rate;
  tty.c_ospeed = baud_rate;

  // Apply settings
  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Flush any stale data
  flush();

  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const
{
  return fd_ >= 0;
}

int SerialPort::read(uint8_t * buffer, size_t max_length)
{
  if (fd_ < 0) {
    return -1;
  }

  ssize_t bytes = ::read(fd_, buffer, max_length);
  if (bytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;  // No data available
    }
    return -1;  // Error
  }

  return static_cast<int>(bytes);
}

int SerialPort::write(const uint8_t * data, size_t length)
{
  if (fd_ < 0) {
    return -1;
  }

  ssize_t bytes = ::write(fd_, data, length);
  return (bytes < 0) ? -1 : static_cast<int>(bytes);
}

void SerialPort::flush()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
}

bool SerialPort::setBaudRate(int baud_rate)
{
  (void)baud_rate;
  // Baud rate is set in open() via termios2
  return true;
}

}  // namespace evarobot_rc
