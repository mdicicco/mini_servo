#include "mini_servo_hardware/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#else
#include <asm/ioctls.h>
#include <asm/termbits.h>
#endif

namespace mini_servo_hardware
{

SerialPort::~SerialPort() { close(); }

bool SerialPort::is_open() const { return fd_ >= 0; }

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::open(const std::string & device, int baud_rate)
{
  close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  struct termios tio;
  if (tcgetattr(fd_, &tio) < 0) {
    close();
    return false;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD | CS8;
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) < 0) {
    close();
    return false;
  }

#ifdef __APPLE__
  speed_t speed = static_cast<speed_t>(baud_rate);
  if (ioctl(fd_, IOSSIOSPEED, &speed) < 0) {
    close();
    return false;
  }
#else
  struct termios2 tio2;
  if (ioctl(fd_, TCGETS2, &tio2) < 0) {
    close();
    return false;
  }
  tio2.c_cflag &= ~CBAUD;
  tio2.c_cflag |= BOTHER;
  tio2.c_ispeed = static_cast<speed_t>(baud_rate);
  tio2.c_ospeed = static_cast<speed_t>(baud_rate);
  tio2.c_cflag |= CLOCAL | CREAD | CS8;
  tio2.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tio2.c_iflag &=
    ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tio2.c_oflag &= ~OPOST;
  tio2.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tio2.c_cc[VMIN] = 0;
  tio2.c_cc[VTIME] = 0;
  if (ioctl(fd_, TCSETS2, &tio2) < 0) {
    close();
    return false;
  }
#endif

  return true;
}

bool SerialPort::write(const std::string & data)
{
  if (!is_open()) {
    return false;
  }

  size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

std::string SerialPort::read_available()
{
  if (!is_open()) {
    return {};
  }

  std::string out;
  char buf[256];
  while (true) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      out.append(buf, static_cast<size_t>(n));
      continue;
    }
    break;
  }
  return out;
}

}  // namespace mini_servo_hardware
