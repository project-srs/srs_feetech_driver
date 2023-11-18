#pragma once
#include <map>
#include <numeric>
#include <optional>

#include "serial_port_handler.hpp"

struct ServoStatus
{
  int position{2048};
  int velocity{0};
};

struct ServoConfig
{
  int min_tick{1024};
  int max_tick{3072};
};

class FeetechHandler
{
public:
  FeetechHandler(void) {}

  bool Initialize(const std::map<int, ServoConfig> & config_list)
  {
    serial_port_handler_.Open("/dev/feetech");
    config_list_ = config_list;
    return true;
  }

  void Close(void) { serial_port_handler_.CloseSerial(); }

  bool SetCommand(const int id, const int position, const int velocity)
  {
    if (!config_list_.count(id)) {
      printf("id[%d] not found in config_list\n", id);
      return false;
    }
    const ServoConfig config = config_list_.at(id);
    int regulated_position = std::min(std::max(position, config.min_tick), config.max_tick);
    int regulated_velocity = std::min(std::max(velocity, 0), max_tick_per_sec_);
    std::vector<unsigned char> send_data =
      GenerateSetPositionVelocityCommand(id, regulated_position, regulated_velocity);
    SendCommandAndWait(send_data);
    return true;
  }

  bool RequestStatus(void)
  {
    for (auto config_pair : config_list_) {
      int id = config_pair.first;
      std::vector<unsigned char> send_data = GenerateRequestPositionVelocityCommand(id);
      SendCommandAndWait(send_data);
    }
    std::vector<unsigned char> buffer = serial_port_handler_.Read();
    std::vector<std::vector<unsigned char>> split_data = SplitInput(buffer);
    for (auto msg : split_data) {
      std::optional<std::pair<int, ServoStatus>> result_opt = ParseInput(msg);
      if (result_opt) {
        status_list_[result_opt.value().first] = result_opt.value().second;
      }
    }
    return true;
  }

  std::optional<ServoStatus> GetStatus(const int id) const
  {
    if (status_list_.count(id)) {
      return status_list_.at(id);
    }
    return std::nullopt;
  }

private:
  bool SendCommandAndWait(const std::vector<unsigned char> & buffer)
  {
    std::vector<unsigned char> send_command;
    send_command.push_back(0xff);
    send_command.push_back(0xff);
    for (auto b : buffer) {
      send_command.push_back(b);
    }
    send_command.push_back(GetCheckSum(buffer));
    bool ret = serial_port_handler_.Write(send_command);
    usleep(4 * 1000);  // wait 4[ms] for finish sending
    return ret;
  }

  static unsigned char GetCheckSum(const std::vector<unsigned char> & buffer)
  {
    int sum = std::accumulate(buffer.begin(), buffer.end(), 0);
    return 0xff - (0xff & sum);
  }

  static std::vector<std::vector<unsigned char>> SplitInput(
    const std::vector<unsigned char> & buffer)
  {
    std::vector<std::vector<unsigned char>> split_output;
    for (size_t i = 0; i + 3 < buffer.size();) {
      bool h0_valid = (buffer[i + 0] == 0xff);
      bool h1_valid = (buffer[i + 1] == 0xff);
      int length = buffer[i + 3];
      bool ln_valid = (size_t)(i + 3 + length) < buffer.size();
      if (h0_valid && h1_valid && ln_valid) {
        unsigned char checksum = buffer[i + 3 + length];

        std::vector<unsigned char> msg;
        for (size_t j = 2; j < (size_t)(length + 3); j++) {
          msg.push_back(buffer[i + j]);
        }
        unsigned char expected_check_sum = GetCheckSum(msg);

        if (checksum == expected_check_sum) {
          split_output.push_back(msg);
        }

        i += (length + 4);
        continue;
      }
      i++;
    }
    return split_output;
  }

  static std::optional<std::pair<int, ServoStatus>> ParseInput(
    const std::vector<unsigned char> & msg)
  {
    if (msg.size() != 7) {
      return std::nullopt;
    }
    std::pair<int, ServoStatus> output;
    // set id
    output.first = msg[0];
    // set position
    output.second.position = msg[4] << 8 | msg[3];
    // set velocity
    uint16_t vel_tmp = msg[6] << 8 | msg[5];
    if (vel_tmp & 0x8000) {
      output.second.velocity = -((int)vel_tmp - 0x8000);
    } else {
      output.second.velocity = vel_tmp;
    }
    return output;
  }

  static std::vector<unsigned char> GenerateSetPositionVelocityCommand(
    const int id, const int pos, const int vel)
  {
    std::vector<unsigned char> buffer;
    buffer.push_back(id);
    buffer.push_back(9);                  // len
    buffer.push_back(3);                  // write command
    buffer.push_back(42);                 // command_pos_time_vel address
    buffer.push_back(0xff & (pos >> 0));  // pos
    buffer.push_back(0xff & (pos >> 8));
    buffer.push_back(0x00);  // time
    buffer.push_back(0x00);
    buffer.push_back(0xff & (vel >> 0));  // vel
    buffer.push_back(0xff & (vel >> 8));
    return buffer;
  }

  static std::vector<unsigned char> GenerateRequestPositionVelocityCommand(const int id)
  {
    std::vector<unsigned char> buffer;
    buffer.push_back(id);
    buffer.push_back(4);     // len
    buffer.push_back(2);     // write command
    buffer.push_back(0x38);  // current_pos
    buffer.push_back(4);
    return buffer;
  }

private:
  SerialPortHandler serial_port_handler_;
  std::map<int, ServoConfig> config_list_;
  std::map<int, ServoStatus> status_list_;
  static constexpr int max_tick_per_sec_ = 4095;
};
