#ifndef _TELEMETRTY_H_
#define _TELEMETRTY_H_

#include "rodos.h"
#include "topics.h"
#include "config_gold.h"
#include "config_mpc.h"
#include "tof_range.h"
#include <stdio.h>
#include <assert.h>

extern HAL_UART WIFI_1;

class telemetry_thread : public Thread
{
private:
  int period = THREAD_PERIOD_TELEMETRY_MILLIS;

public:
  telemetry_thread(const char *thread_name, const int priority) : Thread(thread_name, priority) {}
  telemetry_thread(const char* thread_name, const int32_t priority, const int32_t stackSize ) : Thread(thread_name, priority, stackSize) {}

  void init();
  void run();
};

extern telemetry_thread tamariw_telemetry_thread;

#endif // telemetry.h
