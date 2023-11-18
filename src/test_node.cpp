#include <stdio.h>
#include <unistd.h>
#include <srs_feetech_driver/feetech_handler.hpp>

int main(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  printf("Start\n");
  FeetechHandler feetech_handler;

  std::map<int,ServoConfig> config_list;
  config_list[1] = {1024, 3072};
  config_list[2] = {1024, 2400};

  feetech_handler.Initialize(config_list);
  printf("Command1\n");
  feetech_handler.SetCommand(1, 1024, 0);
  feetech_handler.SetCommand(2, 2048, 0);
  sleep(2);
  printf("Command2\n");
  feetech_handler.SetCommand(1, 2048, 0);
  feetech_handler.SetCommand(2, 1024, 0);
  sleep(2);
  printf("Command3\n");
  feetech_handler.SetCommand(1, 3072, 0);
  feetech_handler.SetCommand(2, 2048, 0);
  sleep(2);

  feetech_handler.RequestStatus();
  printf("Done\n");
  auto s1_opt = feetech_handler.GetStatus(1);
  if (s1_opt) {
    printf("id1, %d %d\n", s1_opt.value().position, s1_opt.value().velocity);
  }
  auto s2_opt = feetech_handler.GetStatus(2);
  if (s2_opt) {
    printf("id2, %d %d\n", s2_opt.value().position, s2_opt.value().velocity);
  }

  return 0;
}