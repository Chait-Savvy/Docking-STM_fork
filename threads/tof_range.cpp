// Performs ToF range measurement and publishes to topic_tof_range

#include "hal.h"
#include "tof.h"
#include "utils.h"
#include "rodos.h"
#include "topics.h"
#include "platform.h"
#include "tof_range.h"

data_tof_range tof_data; // ToF topic

void tof_range_thread::init()
{
  tof::int_xshunt();
}

void tof_range_thread::init_params()
{
  if (tof::init(TOF_IDX_ALL) == TOF_STATUS_OK)
  {
    //PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    //PRINTF("VL53L4CD error :(\n");
  }
  tof::enable_median_filter();
  tof::enable_mean_filter();
}

void tof_range_thread::run()
{
  tof::wakeup();
  init_params();
 

  TIME_LOOP ( THREAD_TOF_START * MILLISECONDS, TIME_PERIOD_PER_THREAD * MILLISECONDS)
  {
    if(restart_tof)
    {
    
      tof::restart();
      init_params();
      restart_tof = false;
    }
    else
    {
      time_thread = NOW();

      // Measure ToF distances and compute velocities.
      tof_status status = tof::get_distance(tof_data.d);
      const double vel_dt = (NOW() - time_vel) / SECONDS;
      tof::get_velocity(tof_data.d, vel_dt, tof_data.v);
      time_vel = NOW();

      // Winsorized mean and approach detection.
      tof_data.dm = winsorized_mean(tof_data.d);
      tof_data.vm = winsorized_mean(tof_data.v);
      tof_data.approach = false;

      // Calculation of x, y and z axises
      tof_data.x_axis = get_x_axis(tof_data.d);
      tof_data.y_axis = get_y_axis(tof_data.d);
      tof_data.z_axis = get_z_axis(tof_data.d);

      // Calculation of roll and pitch
      tof_data.roll = get_roll(tof_data.d);
      tof_data.pitch = get_pitch(tof_data.d);

      // Angular velocities of roll and pitch
      tof_data.w_roll = get_angular_velocity_roll(tof_data.roll, vel_dt, tof_data.w_roll);
      tof_data.w_pitch = get_angular_velocity_pitch(tof_data.pitch, vel_dt, tof_data.w_pitch);

      // Publish data to collision control thread.
      tof_data.status = status;
      tof_data.dt = (NOW() - time_thread) / MILLISECONDS;
      topic_tof_range.publish(tof_data);

      // PRINTF(" %0.2f %0.2f %0.2f %0.2f \n ", tof_data.dm , tof_data.vm  , tof_data.roll , tof_data.w_roll);

      // Restart ToFs if error.
      if(status != TOF_STATUS_OK)
      {
        restart_tof = true;
      }
    }
  }
  //PRINTF("tof thread time : %f \n",(float)((NOW() - time_thread) / MILLISECONDS));
}

// extern tof_range_thread tamariw_tof_range_thread
tof_range_thread tamariw_tof_range_thread("tof_range_thread",100);
