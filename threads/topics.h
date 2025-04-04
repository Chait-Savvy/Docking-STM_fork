#ifndef _TOPICS_H_
#define _TOPICS_H_

#include "rodos.h"

struct data_tof_range 
{
  int d[4]       = {0, 0, 0, 0}; // Distance, mm
  float d_meam[4]= {0.0, 0.0, 0.0, 0.0}; // Mean or average distance
  float v[4]     = {0, 0, 0, 0}; // Velocity, mm/s
  float dm       = 0.0;          // Winsorized mean of distances, mm
  float vm       = 0.0;          // Winsorized mean of velocity, mm/s
  float dt       = 0;            // Thread period, millis
  bool  approach = false;        // True if satellites approach each other.
  int   status   = 0;
  float x_axis   = 0;          // x axis calculation
  float y_axis   = 0;          // y axis calculation
  float z_axis   = 0;          // z axis calculation
  float roll     = 0;          // Roll calculation
  float pitch    = 0;          // Pitch calculation 
  float w_roll   = 0;          // Angular velocity roll
  float w_pitch  = 0;          // Angular velocity pitch
};

struct data_current_ctrl
{
  float i[4]; // Current, milli Amp
  float dt = 0; // Thread period, millis
};

struct data_collision_ctrl
{
  float dk[2]; // Distance gains: {kp, ki}
  float vk[2]; // Velocity gains: {kp, ki}
  float dt = 0; // Thread period, millis
};

struct data_desired_current
{
  float i[4]; // milli Amp
};

typedef enum
{
  //magnets actuation based on phase  
  TCMD_APPROACH,
  TCMD_ALLIGN_CW,
  TCMD_ALLIGN_CCW,
  TCMD_DOCKING_CW,
  TCMD_DOCKING_CCW,

  //magnets on/off
  TCMD_MAGNETS_ON_OFF_FLAG,

  // Must be at last!
  TCMD_LENGTH
} tcommand_t;

typedef struct           // Telecommands from groundstation
{
  float data;            // Value corresponding to tcommand_t
  tcommand_t idx;        // Which parameter has changed?
} tcmd_t;

extern Topic<data_tof_range> topic_tof_range;
// extern Topic<tcmd_struct> topic_tcmd_exe;
// extern Topic<data_current_ctrl> topic_current_ctrl;
// extern Topic<data_collision_ctrl> topic_collision_ctrl;
// extern Topic<data_desired_current> topic_desired_current;
//extern Topic<data_extended_kalman_filter> topic_extended_kalman_filter;
//extern Topic<data_motion_prediction_controller> topic_motion_prediction_controller;

#endif // telecommand.h