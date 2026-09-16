#ifndef MINI_SERVO_HARDWARE__SERIAL_PORT_HPP_
#define MINI_SERVO_HARDWARE__SERIAL_PORT_HPP_

#include <string>

namespace mini_servo_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool open(const std::string & device, int baud_rate);
  void close();
  bool is_open() const;

  bool write(const std::string & data);
  std::string read_available();

private:
  int fd_{-1};
};

}  // namespace mini_servo_hardware

#endif  // MINI_SERVO_HARDWARE__SERIAL_PORT_HPP_
