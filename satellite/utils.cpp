#include "utils.h"
#include "rodos.h"
#include "platform.h"

#include <math.h>

#define DisBtwSenrs 40   //distance between sensors in [mm]

static bool first_roll_velocity = true;
static bool first_pitch_velocity = true;
static float last_roll = 0.0;
static float last_pitch = 0.0;

// Sign of input number
int sign(float in)
{
  if(in < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}

void swap(float *a, float *b)
{
  float temp = *a;
  *a = *b;
  *b = temp;
}

/**
 * @brief Mean of input data by limiting extreme values.
 * @return 0.5 * (b + c) where data is sorted as a < b < c < d.
 */
float winsorized_mean(const float x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) {
    arr[i] = x[i];
  }

  // Sorting the necessary elements to find the middle two
  if (arr[0] > arr[1]) swap(&arr[0], &arr[1]);
  if (arr[2] > arr[3]) swap(&arr[2], &arr[3]);
  if (arr[0] > arr[2]) swap(&arr[0], &arr[2]);
  if (arr[1] > arr[3]) swap(&arr[1], &arr[3]);
  if (arr[1] > arr[2]) swap(&arr[1], &arr[2]);

  return (arr[1] + arr[2]) / 2.0;
}

/**
 * @brief Mean of input data by limiting extreme values.
 * @return 0.5 * (b + c) where data is sorted as a < b < c < d.
 */
float winsorized_mean(const int x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) {
    arr[i] = (float)x[i];
  }

  // Sorting the necessary elements to find the middle two
  if (arr[0] > arr[1]) swap(&arr[0], &arr[1]);
  if (arr[2] > arr[3]) swap(&arr[2], &arr[3]);
  if (arr[0] > arr[2]) swap(&arr[0], &arr[2]);
  if (arr[1] > arr[3]) swap(&arr[1], &arr[3]);
  if (arr[1] > arr[2]) swap(&arr[1], &arr[2]);

  return (arr[1] + arr[2]) / 2.0;
}

float get_x_axis(const int x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) 
  {
    arr[i] = (float)x[i];
  }

  return (arr[0] + arr[1])/2 - (arr[2] + arr[3])/2;
}

float get_y_axis(const int x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) 
  {
    arr[i] = (float)x[i];
  }

  return (arr[0] + arr[3])/2 - (arr[1] + arr[2])/2;
}

float get_z_axis(const int x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) 
  {
    arr[i] = (float)x[i];
  }

  return (arr[0] + arr[1] + arr[2] + arr[3])/4;
}

float get_roll(const int x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) 
  {
    arr[i] = (float)x[i];
  }
  
  return atan2(arr[0] - arr[1], DisBtwSenrs);;
}

float get_pitch(const int x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) 
  {
    arr[i] = (float)x[i];
  }
  
  return atan2(arr[2] - arr[3], DisBtwSenrs);;
}

float get_angular_velocity_roll(const float roll, const double dt, float w_roll)
{
 if(first_roll_velocity)
  {
    last_roll = roll;
    w_roll = 0.0;

    first_roll_velocity = false;
    return 1;
  }

  if(roll != last_roll)
  {
    // Compute velocity
    w_roll = (roll - last_roll) / dt;
    //PRINTF("2 roll: %5.5f\t last_roll: %5.5f\t w_roll: %5.5f\n", roll, last_roll, w_roll);
    last_roll = roll;
  }
  return w_roll;
}

float get_angular_velocity_pitch(const float pitch, const double dt, float w_pitch)
{
 if(first_pitch_velocity)
  {
    last_pitch = pitch;
    w_pitch = 0.0;

    first_pitch_velocity = false;
    return 1;
  }

  if(pitch != last_pitch)
  {
    // Compute velocity
    w_pitch = (pitch - last_pitch) / dt;
    //PRINTF("2 roll: %5.5f\t last_roll: %5.5f\t w_roll: %5.5f\n", roll, last_roll, w_roll);
    last_pitch = pitch;
  }
  return w_pitch;
}