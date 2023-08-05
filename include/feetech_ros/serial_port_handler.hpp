#pragma once

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>

class SerialPortHandler
{
public:
  SerialPortHandler() {}

  bool Open(std::string device_name)
  {
    fd_ = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd_, F_SETFL, 0);

    // load configuration
    struct termios conf_tio;
    tcgetattr(fd_, &conf_tio);
    // set baudrate
    speed_t BAUDRATE;
    BAUDRATE = B1000000;

    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);

    // non canonical, non echo back
    conf_tio.c_lflag &= CS8;
    // conf_tio.c_lflag &= ~(ECHO | ICANON);
    // non blocking
    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;
    // store configuration
    tcsetattr(fd_, TCSANOW, &conf_tio);

    if (fd_ < 0)
    {
      printf("fail to open port\n");
      return false;
    }

    return true;
  }

  void SetVerbose(void) {
    verbose_ = true;
  }

  bool Write(const std::vector<unsigned char> buffer)
  {
    if (fd_ < 0)
    {
      printf("fd not initialized\n");
      return false;
    }
    if (verbose_) {
      printf("write[%lu] ", buffer.size());
      for (auto b : buffer)
      {
        printf("%x ", b);
      }
      printf("\n");
    }

    write(fd_, buffer.data(), (int)buffer.size());
    return true;
  }

  std::vector<unsigned char> Read(void)
  {
    if (fd_ < 0)
    {
      printf("fd not initialized\n");
      return {};
    }
    std::vector<unsigned char> buffer;
    buffer.resize(256);
    int recv_num = read(fd_, buffer.data(), buffer.size());
    if (0 < recv_num)
    {
      if (verbose_) {
        printf("recv[%lu] ", (size_t)recv_num);
        for (size_t i = 0; i < (size_t)recv_num; i++)
        {
          printf("%x ", buffer[i]);
        }
        printf("\n");
      }
    }
    buffer.resize(recv_num);
    return buffer;
  }

  void CloseSerial(void) { close(fd_); }

private:
  int fd_{0};
  bool verbose_{false};
};
