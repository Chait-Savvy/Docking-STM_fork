// TAMARIW telecommands only consists of (character, number) pair.
// The intended numbers for each characters are in the comments below.

#ifndef __TELECOMMAND_H__
#define __TELECOMMAND_H_

#include <stdlib.h>
#include "rodos.h"
#include "pid.h"
#include "led.h"
#include "magnet.h"
#include "satellite_config.h"
#include "config_mpc.h"
#include "topics.h"
#include "telemetry.h"

#define TELECOMMAND_MAX_LEN  12

// Frame markers
#define TELECOMMAND_START '$'
#define TELECOMMAND_STOP '#'
#define TELECOMMAND_DELIMITTER ','


// HAL_UART WIFI_1(UART_IDX1, GPIO_009, GPIO_010);
HAL_UART WIFI_1(UART_IDX3, GPIO_027, GPIO_026, GPIO_INVALID, GPIO_INVALID);


class telecommand_thread: public Thread
{
private:

  // int period = THREAD_PERIOD_TELECOMMAND_MILLIS;
  tcmd_t tcmd_data;
  char tcmd_msg[25];
  bool Magnet_safety_flag = false;

public:
  telecommand_thread(const char *thread_name, const int priority) : Thread(thread_name, priority) {}
  telecommand_thread(const char *thread_name, const int priority , const int stack_size) : Thread(thread_name,priority,stack_size){}
  bool parse_tcmd(const char *msg, tcommand_t *cmd, float *data);
  bool execute_command(tcommand_t cmd, float data, tcmd_t *tcmd);
  float pid_ctrl_magnets(int idx,float data);
  double sign(const double in);
  void init();
  void run();
};

extern telecommand_thread tamariw_telecommand_thread;

#endif /* telecommand.h */
