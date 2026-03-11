// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: POSIX serial port handler for CRSF/ELRS communication

#ifndef EVAROBOT_RC__SERIAL_PORT_HPP_
#define EVAROBOT_RC__SERIAL_PORT_HPP_

#include <string>
#include <cstdint>

namespace evarobot_rc
{

/**
 * @brief POSIX serial port wrapper for CRSF communication
 *
 * Handles opening, configuring, reading, and closing a serial port
 * using termios. Supports the non-standard 420000 baud rate used
 * by ExpressLRS CRSF protocol.
 */
class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  // Non-copyable
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /**
   * @brief Open and configure the serial port
   * @param device Path to serial device (e.g. "/dev/ttyAMA0")
   * @param baud_rate Baud rate (default 420000 for CRSF)
   * @return true if successful
   */
  bool open(const std::string & device, int baud_rate = 420000);

  /**
   * @brief Close the serial port
   */
  void close();

  /**
   * @brief Check if port is open
   */
  bool isOpen() const;

  /**
   * @brief Read available bytes from serial port (non-blocking)
   * @param buffer Output buffer
   * @param max_length Maximum bytes to read
   * @return Number of bytes read, or -1 on error
   */
  int read(uint8_t * buffer, size_t max_length);

  /**
   * @brief Write bytes to serial port
   * @param data Data buffer
   * @param length Number of bytes to write
   * @return Number of bytes written, or -1 on error
   */
  int write(const uint8_t * data, size_t length);

  /**
   * @brief Flush input and output buffers
   */
  void flush();

  /**
   * @brief Get the device path
   */
  const std::string & getDevice() const { return device_; }

private:
  /**
   * @brief Convert integer baud rate to termios speed constant
   */
  bool setBaudRate(int baud_rate);

  int fd_;
  std::string device_;
};

}  // namespace evarobot_rc

#endif  // EVAROBOT_RC__SERIAL_PORT_HPP_
