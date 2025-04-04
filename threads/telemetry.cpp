
#include "telemetry.h"

#define DELIMITTER " "

HAL_UART WIFI_1(UART_IDX3, GPIO_027, GPIO_026);

Topic<data_tof_range> topic_tof_range(-1, "data_tof_range");
CommBuffer<data_tof_range> cb_tof;
Subscriber subs_tof(topic_tof_range, cb_tof);
static data_tof_range rx_tof;

static double time = NOW();
static float dt = 0;
static char msg_fin[90];
static int check_tof = 999;


void telemetry_thread::init()
{
  WIFI_1.init(115200);
}

void telemetry_thread::run()
{
  //TIME_LOOP (period * MILLISECONDS * 5, period * MILLISECONDS * 5)
  // TIME_LOOP (THREAD_EKF_START * MILLISECONDS, TIME_PERIOD_PER_THREAD * MILLISECONDS)
  // {
    time = NOW();
    cb_tof.getOnlyIfNewData(rx_tof);

    // SNPRINTF(msg_fin, sizeof(msg_fin), "\n%d,%d %d %d %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d",
    //   check_tof,
    //   rx_tof.d[0], rx_tof.d[1], rx_tof.d[2], rx_tof.d[3],
    //   telemetry_i_rx.i_dt_status[0], telemetry_i_rx.i_dt_status[1], telemetry_i_rx.i_dt_status[2], telemetry_i_rx.i_dt_status[3],
    //   rx_tof.x_axis,
    //   telemetry_i_rx.desired_current, rx_tof.w_roll,
    //   *telemetry_ekf_data_temp.ekf_Xk.r[0],*telemetry_ekf_data_temp.ekf_Xk.r[1],*telemetry_ekf_data_temp.ekf_Xk.r[2],*telemetry_ekf_data_temp.ekf_Xk.r[3],
    //   check_tof
    // );

    
    SNPRINTF(msg_fin, sizeof(msg_fin), "\n%d"DELIMITTER"%0.2f"DELIMITTER"%0.2f"DELIMITTER"%0.2f"DELIMITTER"%0.2f"DELIMITTER"%d",
      check_tof,
      rx_tof.dm,rx_tof.vm,rx_tof.roll,rx_tof.w_roll,  
      check_tof
    );

    PRINTF("%s", msg_fin);//

    WIFI_1.write(msg_fin, sizeof(msg_fin));

    dt =  (NOW() - time) / MILLISECONDS;
  // }
}

// telemetry_thread tamariw_telemetry_thread("telemetry_thread", THREAD_PRIO_TELEMETRY, 4000);
extern telemetry_thread tamariw_telemetry_thread;
