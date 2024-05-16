
#include <math.h>

#include "tof.h"
#include "rodos.h"
#include "utils.h"
#include "MedianFilter.h"
#include "VL53L4CD_api.h"
#include "platform_TAMARIW.h"
#include "VL53L4CD_calibration.h"

bool tof_filter_flag = false;
MedianFilter<int, 25> filter[4];
float last_distance[4];

// VL53L4CD API params
VL53L4CD_ResultsData_t tof_result;
bool i2c_init_flag = false;

void tof::enable_median_filter(void)
{
  tof_filter_flag = true;
}

void tof::disable_median_filter(void)
{
  tof_filter_flag = false;
}

// Initialize a single sensor
tof_status init_single(const tof_idx idx)
{
  // One I2C init sufficies
  if (!i2c_init_flag)
  {
    init4cd();
    i2c_init_flag = true;
  }

  // Select sensor using MUX
  PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS);

  // Initialize and return status
  if (VL53L4CD_SensorInit(TOF_I2C_ADDRESS) == VL53L4CD_ERROR_NONE)
  {
    return TOF_STATUS_OK;
  }

  return TOF_STATUS_ERROR;
}

// Initialize either single or all sensors
tof_status tof::init(const tof_idx idx)
{
  if (idx != TOF_IDX_ALL)
  {
    return init_single(idx);
  }

  // Test all sensors
  for (uint8_t i = TOF_IDX_0; i <= TOF_IDX_3; i++)
  {
    if (init_single((tof_idx)i) == TOF_STATUS_ERROR)
    {
      return TOF_STATUS_ERROR;
    }
  }
   VL53L4CD_SetRangeTiming(TOF_I2C_ADDRESS, 10, 0);

  return TOF_STATUS_OK;
}

// Range in mm for a 'single' ToF
// A guide to using the VL53L4CD ultra lite driver (UM2931): Figure 7
tof_status tof::get_single_distance(const tof_idx idx, int *distance)
{
  if (idx == TOF_IDX_ALL)
  {
    return TOF_STATUS_ERROR;
  }

  PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS);

  if (VL53L4CD_StartRanging(TOF_I2C_ADDRESS) == VL53L4CD_ERROR_NONE)
  {

    // Wait for data ready
    while (1)
    {
      uint8_t data_ready = 0;
      VL53L4CD_CheckForDataReady(TOF_I2C_ADDRESS, &data_ready);

      if (data_ready)
      {
        break;
      }
    }

    VL53L4CD_GetResult(TOF_I2C_ADDRESS, &tof_result);

    if (tof_filter_flag)
    {
      filter[(uint8_t)idx].addSample(tof_result.distance_mm);
      *distance = filter[(uint8_t)idx].getMedian();
    }
    else
    {
      *distance = tof_result.distance_mm;
    }

    VL53L4CD_ClearInterrupt(TOF_I2C_ADDRESS);
    VL53L4CD_StopRanging(TOF_I2C_ADDRESS);

    return TOF_STATUS_OK;
  }

  return TOF_STATUS_ERROR;
}

// Range in mm for 'all' sensors
tof_status tof::get_distance(int distance[4])
{
  int temp_dist;

  for (uint8_t i = TOF_IDX_0; i <= TOF_IDX_3; i++)
  {
    if (get_single_distance((tof_idx)i, &temp_dist) == TOF_STATUS_OK)
    {
      distance[i] = temp_dist;
      AT(5 * MILLISECONDS);
    }
    else
    {
      return TOF_STATUS_ERROR;
    }
  }

  return TOF_STATUS_OK;
}

/*
  Calibrates four ToFs for input target distance using n samples.
  The offsets are written to VL53L4CD_RANGE_OFFSET_MM register of ToF.

  ST's recommendation:
     _____________________________
    | setting   | min |  ST | max |
    |-----------|-----|-----|-----|
    | target_mm | 10  | 100 | 255 |
    |     n     |  5  | 20  | 255 |
    |___________|_____|_____|_____|
*/
tof_status tof::calibrate(const int16_t target_mm, const int16_t n)
{
  for (uint8_t i = TOF_IDX_0; i <= TOF_IDX_3; i++)
  {
    PCA9546_SelPort(i, TOF_I2C_MUX_ADDRESS);

    int16_t offset, old_offset;
    VL53L4CD_GetOffset(TOF_I2C_ADDRESS, &old_offset);

    if (VL53L4CD_CalibrateOffset(TOF_I2C_ADDRESS, target_mm, &offset, n) != VL53L4CD_ERROR_NONE)
    {
      return TOF_STATUS_ERROR;
    }

    PRINTF("ToF %d: Offset changed from %d to %d.\n", (uint8_t)i, old_offset, offset);
  }

  return TOF_STATUS_OK;
}

// Relative yaw between suspended spacecrafts
tof_status tof::get_yaw(float *yaw)
{
  int distance[4];

  if (tof::get_distance(distance) == TOF_STATUS_OK)
  {
    *yaw =  R2D * atan2(distance[0] - distance[2], TOF_DIMENSION_WIDTH_MM);

    return TOF_STATUS_OK;
  }

  return TOF_STATUS_ERROR;
}

// Relative velocity wrt. to the satellite
tof_status tof::get_velocity(float velocity[4])
{
  int distance[4];

  if (tof::get_distance(distance) == TOF_STATUS_OK)
  {
    for(uint8_t i = 0; i < 4; i++)
    {
      velocity[i] = (distance[i] - last_distance[i]) / PERIOD_TOF_MILLIS;
      last_distance[i] = distance[i];
    }

    return TOF_STATUS_OK;
  }

  return TOF_STATUS_ERROR;
}
